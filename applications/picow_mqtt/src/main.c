#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/net/mqtt.h>


#include <zephyr/logging/log.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>

LOG_MODULE_REGISTER(MAIN);

//STA-mode WiFi Handler event mask
#define NET_EVENT_WIFI_MASK (NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

//MQTT Event Handler Mask
#define NET_EVENT_MQTT_MASK (MQTT_EVT_CONNACK | MQTT_EVT_PUBLISH | MQTT_EVT_SUBACK| MQTT_EVT_DISCONNECT)    

//Server and Port definitions
#define SERVER_IP_ADDR CONFIG_NET_CONFIG_MY_IPV4_ADDR //Linux Host Lab IP Address
#define SERVER_PORT 4242

#define MESSAGE "Hello from the Pico W\r\n"
#define PING_INTERVAL K_SECONDS(5)
#define POLL_TIMEOUT -1 //(ms) to regulate timeout of non-blocking receive
#define CLIENT_ID "rts_picow"

/* STA Mode interface and WiFi params */
static struct net_if *sta_iface;
static struct wifi_connect_req_params sta_config;

//Net Management event callback definition
static struct net_mgmt_event_callback cb;

//Buffers and context for MQTT
static uint8_t rx_buffer[256];
static uint8_t tx_buffer[256];
static struct mqtt_client client_ctx;
static struct sockaddr_storage broker;
static volatile bool connected = false;


/* Function to handle wifi events */
static void wifi_event_handler(struct net_mgmt_event_callback *cb, uint32_t mgmt_event,
			       struct net_if *iface)
{
    switch (mgmt_event) {
    case NET_EVENT_WIFI_CONNECT_RESULT: {
        LOG_INF("Connected to %s", sta_config.ssid);
        break;
    }
    case NET_EVENT_WIFI_DISCONNECT_RESULT: {
        LOG_INF("Disconnected from %s", sta_config.ssid);
        break;
    }
    default:
        LOG_INF("Unhandled WiFi Event, check Event Mask - NET_EVENT_WIFI_MASK!");
        break;
    }
}

/* Event Handler for MQTT */
void mqtt_event_handler(struct mqtt_client *client, const struct mqtt_evt *evt) {
    switch(evt->type) {
        case MQTT_EVT_CONNACK: {
            LOG_INF("MQTT_EVT_CONACK: Connection Established");
            connected = true;
            break;
        }
        case MQTT_EVT_PUBLISH: {
            LOG_INF("Message Published");
            break;
        }
        case MQTT_EVT_SUBACK: {
            LOG_INF("Subscribed to topic");
            break;
        }
        case MQTT_EVT_DISCONNECT: {
            LOG_INF("MQTT Connection disconnected");
        }
        default:
            LOG_INF("Unhandled MQTT Event, check Event Mask - NET_EVENT_MQTT_MASK!");
            break;
    }
}


/* Connect to WiFi in STA mode */
static int connect_to_wifi(void)
{
    if (!sta_iface) {
        LOG_INF("STA: interface no initialized");
        return -EIO;
    }

    //Configure WiFi parameters
    sta_config.ssid = CONFIG_WIFI_CREDENTIALS_STATIC_SSID;
    sta_config.ssid_length = strlen(sta_config.ssid);
    sta_config.psk = CONFIG_WIFI_CREDENTIALS_STATIC_PASSWORD;
    sta_config.psk_length = strlen(sta_config.psk);
    sta_config.security = WIFI_SECURITY_TYPE_WPA_PSK;
    sta_config.channel = WIFI_CHANNEL_ANY;
    sta_config.band = WIFI_FREQ_BAND_2_4_GHZ;

    LOG_INF("Connecting to SSID: %s\n", sta_config.ssid);
    int ret = net_mgmt(NET_REQUEST_WIFI_CONNECT, sta_iface, &sta_config,
                sizeof(struct wifi_connect_req_params));
    if (ret) {
        LOG_ERR("Unable to Connect to (%s)", sta_config.ssid);
    }
    return ret;
}

/* Function to disconnect from WiFi */
static void disconnect_from_wifi(void)
{
    if (!sta_iface) {
        LOG_INF("STA: interface not initialized");
        return;
    }

    int ret = net_mgmt(NET_REQUEST_WIFI_DISCONNECT, sta_iface, NULL, 0);
    if (ret) {
        LOG_ERR("Unable to disconnect from WiFi, errno: %d", ret);
    } else {
        LOG_INF("Successfully disconnected from WiFi");
    }
}

static int broker_init(void)
{
    int ret;
    struct addrinfo *result;
    struct addrinfo *addr;
    struct addrinfo hints = {
        .ai_family = AF_INET,
        .ai_socktype = SOCK_STREAM
    };

    ret = getaddrinfo(CONFIG_MQTT_BROKER_HOSTNAME, NULL, &hints, &result);
    if(ret){
        LOG_ERR("getaddrinfo failed: %d", ret);
        return -ECHILD;
    }
    addr = result;

    /*Look for Broker Address*/
    while(addr != NULL) {
        /*IPv4 Address*/
        if(addr->ai_addrlen == sizeof(struct sockaddr_in)){
            struct sockaddr_in *broker4 = ((struct sockaddr_in *)&broker);
            char ipv4_addr[NET_IPV4_ADDR_LEN];

            broker4->sin_addr.s_addr = ((struct sockaddr_in *)addr->ai_addr)->sin_addr.s_addr;
            broker4->sin_family = AF_INET;
            broker4->sin_port = htons(CONFIG_MQTT_BROKER_PORT);

            inet_ntop(AF_INET, &broker4->sin_addr.s_addr, ipv4_addr, sizeof(ipv4_addr));
            LOG_INF("IPv4 Address found %s", (char *)(ipv4_addr));

            break;
        } else {
            LOG_ERR("ai_addrlen = %u should be %u or %u",
                    (unsigned int)addr->ai_addrlen,
                    (unsigned int)sizeof(struct sockaddr_in),
                    (unsigned int )sizeof(struct sockaddr_in6));
        }

        addr = addr->ai_next;
    }

    freeaddrinfo(result);

    return ret;
}

/*
//Fallback connectivity test block: manually set HiveMQ broker IP
//Useful when DNS resolution fails; IP obtained using 'nslookup broker.hivemq.com'

static int broker_init(void) {
    struct sockaddr_in *broker4 = (struct sockaddr_in *)&broker;

    memset(&broker, 0, sizeof(broker));  // Clear broker struct

    broker4->sin_family = AF_INET;
    broker4->sin_port = htons(1883); // Default MQTT port

    // Replace this IP with one resolved from broker.hivemq.com on your PC
    const char *broker_ip = ""; // find using 'nslookup broker.hivemq.com 

    int ret = inet_pton(AF_INET, broker_ip, &broker4->sin_addr);
    if (ret <= 0) {
        LOG_ERR("inet_pton failed for IP %s, ret: %d", broker_ip, ret);
        return -EINVAL;
    }
    return 0;
}
*/

/* Function to Initialize and Configure the MQTT Client*/
/*
The struct of type `mqtt_client` maintains information relevant to the client,
and is passed to all subsequent calls relevant to the client, z.B., subscribing 
or publishing to topics.
*/
static int begin_mqtt(struct mqtt_client *client)
{
    int err;

    /* initializes the client instance. */
    mqtt_client_init(client);  
    
    /* Resolves the configured hostname and initializes the MQTT broker structure */
    err = broker_init();
    if (err) {
        LOG_ERR("Failed to initialize broker connection with code: %d", err);
        return err;
    }  
    
    /* MQTT client configuration */
    client->broker = &broker;
    client->evt_cb = mqtt_event_handler;
    client->client_id.utf8 = (uint8_t *)CLIENT_ID;
    client->client_id.size = sizeof(CLIENT_ID) - 1;
    client->password = NULL;
    client->user_name = NULL;
    client->protocol_version = MQTT_VERSION_3_1_1;  
    client->transport.type = MQTT_TRANSPORT_NON_SECURE; 
    
    /* MQTT buffers configuration */
    client->rx_buf = rx_buffer;
    client->rx_buf_size = sizeof(rx_buffer);
    client->tx_buf = tx_buffer;
    client->tx_buf_size = sizeof(tx_buffer);  

    return err;
}

int main(void)
{
    int ret = 1;
    int attempts = 0;

    //Instantiate the CDC ACM serial console
    const struct device *usb_acm = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));

    uint32_t dtr = 0;
    if (usb_enable(NULL)) {
        return 0;
    }

    // Pause execution until serial console DTR (Data Terminal Ready) signal is detected
    while (!dtr) {
        uart_line_ctrl_get(usb_acm, UART_LINE_CTRL_DTR, &dtr);
        k_sleep(K_MSEC(100));
    }
    LOG_INF("USB CDC ACM device ready");

    //Register and Add Callback for WiFi connectivity
    net_mgmt_init_event_callback(&cb, wifi_event_handler, NET_EVENT_WIFI_MASK);
    net_mgmt_add_event_callback(&cb);

    //Register and Add Callback for MQTT application layer --> NOT REQUIRED --> done inside client_init

    /* Get STA Interface*/
    sta_iface = net_if_get_wifi_sta();

    // Initialize the WiFi connection with Station (STA) mode - attempt to reconnect thrice
    do {
        ret = connect_to_wifi();
        attempts++;
        LOG_INF("Attempt %d to connect to Wi-Fi...", attempts);
        if (ret != 0) {
            k_sleep(K_SECONDS(2));
        }
    } while (ret != 0 && attempts < 3);

    if (ret == 0) {
        LOG_INF("Successfully connected to Wi-Fi!");
    } else {
        LOG_ERR("Failed to connect to Wi-Fi after 3 attempts. Exiting program.");
        return -1;
    }

    //Wait for a few seconds then initiate the necessary operation
    k_sleep(K_MSEC(1000));

    //Connect to MQTT broker

    ret = begin_mqtt(&client_ctx);
    if (ret) {
        LOG_ERR("MQTT Initialization failed with error code: %d", ret);
        return -1;
    } else {
        LOG_INF("Initializing MQTT client");
    }

    ret = mqtt_connect(&client_ctx);
    if (ret) {
        LOG_ERR("MQTT connection failed with error code: %d", ret);
        return -1;
    } else {
        LOG_INF("MQTT connection request sent");
    }

    struct pollfd fds[1];
    fds[0].fd = client_ctx.transport.tcp.sock;
    fds[0].events = ZSOCK_POLLIN;

    LOG_INF("Waiting for response from MQTT broker...");
    int poll_ret = poll(fds, 1, 5000);
    if (poll_ret < 0) {
        LOG_ERR("Poll error: %d", poll_ret);
    } else if (poll_ret == 0) {
        LOG_WRN("Poll timed out");
    } else {
        LOG_INF("Poll returned, handling MQTT input");
        mqtt_input(&client_ctx);
    }

    if (!connected) {
        LOG_ERR("MQTT connection not established, aborting...");
        mqtt_abort(&client_ctx);
    } else {
        LOG_INF("MQTT connection successfully established");
    }
   
    disconnect_from_wifi();
    return 0;
}