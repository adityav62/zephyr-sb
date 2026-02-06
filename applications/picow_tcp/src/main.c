#include <zephyr/kernel.h>
#include <zephyr/usb/usb_device.h>
#include <zephyr/drivers/uart.h>

#include <zephyr/net/wifi.h>
#include <zephyr/net/wifi_mgmt.h>
#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/socket.h>
#include <zephyr/logging/log.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/select.h>

LOG_MODULE_REGISTER(MAIN);

//STA-mode WiFi Handler event mask
#define NET_EVENT_WIFI_MASK                                                                        \
	(NET_EVENT_WIFI_CONNECT_RESULT | NET_EVENT_WIFI_DISCONNECT_RESULT)

//Server and Port definitions
#define SERVER_IP_ADDR CONFIG_NET_CONFIG_MY_IPV4_ADDR //Linux Host Lab IP Address
#define SERVER_PORT 4242

#define MESSAGE "Hello from the Pico W\r\n"
#define PING_INTERVAL K_SECONDS(5)
#define POLL_TIMEOUT 10000 //(ms) to regulate timeout of non-blocking receive

/* STA Mode interface and WiFi params */
static struct net_if *sta_iface;
static struct wifi_connect_req_params sta_config;

//Net Management event callback definition
static struct net_mgmt_event_callback cb;

/* Define Modes for Sending, Receiving and Bi-Directional communication */
enum Mode { SEND, RECEIVE_NBL, RECEIVE_BL, BOTH };
static enum Mode mode = SEND; //Default Value

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

/* Send a message to device on the same network using TCP socket*/
static void send_message_to_pc(void)
{
    int sock, err;
    struct sockaddr_in addr;

    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket: %d", errno);
        return;
    }

    // Initialize server address; Server -> PC; Client -> Pico W
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT);
    err = inet_pton(AF_INET, SERVER_IP_ADDR, &addr.sin_addr);
    if (err != 1) {
        LOG_ERR("Invalid address: %s", SERVER_IP_ADDR);
        close(sock);
        return;
    }

    // Connect to server
    err = connect(sock, (struct sockaddr *)&addr, sizeof(addr));
    if (err < 0) {
        LOG_ERR("Connect failed: %d", errno);
        close(sock);
        return;
    } else {
        LOG_INF("Connected to server");
    }
    
    // Send a message over server
    err = send(sock, MESSAGE, strlen(MESSAGE), 0);
    if (err < 0) {
        LOG_ERR("Failed to send message: %d", errno);
    } else {
        LOG_INF("Successfully sent message: %s", MESSAGE);
    }

    if (mode == BOTH) {
        char recv_buf[128];
        LOG_INF("Waiting for Response from Server...");
        int received = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
        if (received < 0) {
            LOG_ERR("Error in receiving: %d", errno);
        } else if (received == 0) {
            LOG_INF("Connection closed by Server");
        } else {
            recv_buf[received] = '\0'; // Null-terminate the received data
            LOG_INF("Received message from Server: %s", recv_buf);
        }
    }

    close(sock);
}

/* Receive a message from device on the same network using TCP socket*/
static void receive_message_from_pc(void)
{
    int sock;
    struct sockaddr_in addr;
    char recv_buf[128];

    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket: %d", errno);
        return;
    }

    // Retrieve the IP address assigned to Pico W using net_if_get_config
    struct net_if_config *config = net_if_get_config(sta_iface);
    if (!config || !config->ip.ipv4) {
        LOG_ERR("No IPv4 address found in config");
        close(sock);
        return;
    }
    struct net_if_ipv4 *ipv4 = config->ip.ipv4;
    struct in_addr *ip_addr = &ipv4->unicast[0].ipv4.address.in_addr;

    // Set SO_REUSEADDR option - enables re-use of the same IP - useful while testing
    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        LOG_ERR("setsockopt(SO_REUSEADDR) failed: %d", errno);
        close(sock);
        return;
    }

    // Initialize server address; Server -> Pico W; Client -> PC
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT); //htons -> host-to-network short
    addr.sin_addr = *ip_addr;

    // Bind the server socket to the specific IP address
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Failed to bind server socket: %d", errno);
        close(sock);
        return;
    } else {
        LOG_INF("Server bound to IP address %s", inet_ntoa(*ip_addr));
    }

    // Listen for incoming connections
    if (listen(sock, 1) < 0) {
        LOG_ERR("Failed to listen on server socket: %d", errno);
        close(sock);
        return;
    } else {
        LOG_INF("Server is listening on port %d", SERVER_PORT);
    }

    // Set socket to non-blocking mode, requires timeout configuration
    /* By obtaining the existing flags first, we ensure that 
    we do not inadvertently alter other settings of the socket, 
    maintaining any configurations that were previously applied. 
    This practice is essential for robust and reliable socket programming.*/
    int flags = fcntl(sock, F_GETFL, 0);
    if (flags == -1) {
        LOG_ERR("Failed to get socket flags: %d", errno);
        close(sock);
        return;
    }
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) == -1) {
        LOG_ERR("Failed to set socket to non-blocking mode: %d", errno);
        close(sock);
        return;
    }

    //Set Timeout options
    struct timeval tv;
    tv.tv_sec = 10;  // 10 second timeout
    tv.tv_usec = 0;
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(sock, &readfds);

    int select_ret = select(sock + 1, &readfds, NULL, NULL, &tv);
    if (select_ret == -1) {
        LOG_ERR("Select error: %d", errno);
        close(sock);
        return;
    } else if (select_ret == 0) {
        LOG_ERR("Timeout waiting for connection");
        close(sock);
        return;
    }

    //Accept connections on socket
    int client_sock = accept(sock, NULL, NULL);
    if (client_sock < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK) {
            LOG_ERR("Failed to accept connection: %d", errno);
            close(sock);
            return;
        }
    } else {
        LOG_INF("Accepted connection from client");
        close(sock);  // Close the listening socket
        sock = client_sock; // Use the client socket for receiving data
    }
    
    // Set up poll for receiving data
    struct pollfd fds[1];
    fds[0].fd = sock;
    fds[0].events = POLLIN;

    int poll_ret = poll(fds, sizeof(fds), POLL_TIMEOUT);
    if (poll_ret == 0) {
        LOG_ERR("Receive timed out");
        close(sock);
        return;
    } else if (poll_ret < 0) {
        LOG_ERR("Poll error: %d", errno);
        close(sock);
        return;
    } else {
        if (fds[0].revents & POLLIN) {
            // Receive response
            int received = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
            if (received < 0) {
                LOG_ERR("Socket error: %d", errno);
            } else if (received == 0) {
                LOG_ERR("Connection closed by client");
            } else {
                recv_buf[received] = '\0';
                LOG_INF("Received: %s", recv_buf);
            }
        }
    }
    close(sock);
}

static void receive_message_from_pc_bl(void) {
    int sock, client_sock;
    struct sockaddr_in addr;
    char recv_buf[128];

    // Create socket
    sock = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
    if (sock < 0) {
        LOG_ERR("Failed to create socket: %d", errno);
        return;
    }

    // Retrieve the IP address assigned to Pico W using net_if_get_config
    struct net_if_config *config = net_if_get_config(sta_iface);
    if (!config || !config->ip.ipv4) {
        LOG_ERR("No IPv4 address found in config");
        close(sock);
        return;
    }
    struct net_if_ipv4 *ipv4 = config->ip.ipv4;
    struct in_addr *ip_addr = &ipv4->unicast[0].ipv4.address.in_addr;

    // Set SO_REUSEADDR option - enables re-use of the same IP - useful while testing
    int opt = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        LOG_ERR("setsockopt(SO_REUSEADDR) failed: %d", errno);
        close(sock);
        return;
    }

    // Initialize server address; Server -> Pico W; Client -> PC
    addr.sin_family = AF_INET;
    addr.sin_port = htons(SERVER_PORT); //htons -> host-to-network short
    addr.sin_addr = *ip_addr;

    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        LOG_ERR("Failed to bind server socket: %d", errno);
        close(sock);
        return;
    } else {
        LOG_INF("Server bound to IP address %s", inet_ntoa(*ip_addr));
    }

    // Listen for incoming connections
    if (listen(sock, 1) < 0) {
        LOG_ERR("Failed to listen on server socket: %d", errno);
        close(sock);
        return;
    } else {
        LOG_INF("Server is listening on port %d", SERVER_PORT);
    }

    // Accept incoming connection
    client_sock = accept(sock, NULL, NULL);
    if (client_sock < 0) {
        LOG_ERR("Failed to accept connection: %d", errno);
        close(sock);
        return;
    } else {
        LOG_INF("Accepted connection from client");
    }
    close(sock);
    sock = client_sock;

    // Receive message
    int received = recv(sock, recv_buf, sizeof(recv_buf) - 1, 0);
    if (received < 0) {
        LOG_ERR("Failed to receive message: %d", errno);
    } else if (received == 0) {
        LOG_ERR("Connection closed by client");
    } else {
        recv_buf[received] = '\0';
        LOG_INF("Received: %s", recv_buf);
    }
    close(sock);
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

	k_sleep(K_SECONDS(5)); //need to test if this is required
	
	//Register and Add Callback
	net_mgmt_init_event_callback(&cb, wifi_event_handler, NET_EVENT_WIFI_MASK);
	net_mgmt_add_event_callback(&cb);

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
    k_sleep(K_SECONDS(2));
    mode = RECEIVE_NBL;
    for (int i = 0; i < 5; i++){
        switch(mode){
            case SEND: {
                send_message_to_pc();
                break;
            }
            case BOTH: {
                send_message_to_pc();
                break;
            }
            case RECEIVE_NBL: {
                receive_message_from_pc();
                break;
            }
            case RECEIVE_BL: {
                receive_message_from_pc_bl();
                break;
            }
        }
        k_sleep(K_SECONDS(2));
    }
    disconnect_from_wifi();
    return 0;
}