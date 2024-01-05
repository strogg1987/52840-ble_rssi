#include <zephyr.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <sys/byteorder.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_vs.h>

#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/hrs.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS 1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0 DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0 ""
#define PIN 0
#define FLAGS 0
#endif

static struct bt_conn *default_conn;
static uint16_t default_conn_handle;

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(BT_UUID_HRS_VAL)),
};

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
#define DEVICE_BEACON_TXPOWER_NUM 8

static struct k_thread pwr_thread_data;
static K_THREAD_STACK_DEFINE(pwr_thread_stack, 512);

static const int8_t txpower[DEVICE_BEACON_TXPOWER_NUM] = {4, 0, -3, -8,
                                                          -15, -18, -23, -30};
static const struct bt_le_adv_param *param =
    BT_LE_ADV_PARAM(BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_NAME,
                    0x0020, 0x0020, NULL);
const struct device *dev;
static void read_conn_rssi(uint16_t handle, int8_t *rssi)
{
    struct net_buf *buf, *rsp = NULL;
    struct bt_hci_cp_read_rssi *cp;
    struct bt_hci_rp_read_rssi *rp;

    int err;

    buf = bt_hci_cmd_create(BT_HCI_OP_READ_RSSI, sizeof(*cp));
    if (!buf)
    {
        printk("Unable to allocate command buffer\n");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);

    err = bt_hci_cmd_send_sync(BT_HCI_OP_READ_RSSI, buf, &rsp);
    if (err)
    {
        uint8_t reason = rsp ? ((struct bt_hci_rp_read_rssi *)rsp->data)->status : 0;
        printk("Read RSSI err: %d reason 0x%02x\n", err, reason);
        return;
    }

    rp = (void *)rsp->data;
    *rssi = rp->rssi;

    net_buf_unref(rsp);
}

static void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
{
    struct bt_hci_cp_vs_write_tx_power_level *cp;
    struct bt_hci_rp_vs_write_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    int err;

    buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
                            sizeof(*cp));
    if (!buf)
    {
        printk("Unable to allocate command buffer\n");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    cp->handle_type = handle_type;
    cp->tx_power_level = tx_pwr_lvl;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
                               buf, &rsp);
    if (err)
    {
        uint8_t reason = rsp ? ((struct bt_hci_rp_vs_write_tx_power_level *)
                                    rsp->data)
                                   ->status
                             : 0;
        printk("Set Tx power err: %d reason 0x%02x\n", err, reason);
        return;
    }

    rp = (void *)rsp->data;
    printk("Actual Tx Power: %d\n", rp->selected_tx_power);

    net_buf_unref(rsp);
}

static void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
{
    struct bt_hci_cp_vs_read_tx_power_level *cp;
    struct bt_hci_rp_vs_read_tx_power_level *rp;
    struct net_buf *buf, *rsp = NULL;
    int err;

    *tx_pwr_lvl = 0xFF;
    buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
                            sizeof(*cp));
    if (!buf)
    {
        printk("Unable to allocate command buffer\n");
        return;
    }

    cp = net_buf_add(buf, sizeof(*cp));
    cp->handle = sys_cpu_to_le16(handle);
    cp->handle_type = handle_type;

    err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
                               buf, &rsp);
    if (err)
    {
        uint8_t reason = rsp ? ((struct bt_hci_rp_vs_read_tx_power_level *)
                                    rsp->data)
                                   ->status
                             : 0;
        printk("Read Tx power err: %d reason 0x%02x\n", err, reason);
        return;
    }

    rp = (void *)rsp->data;
    *tx_pwr_lvl = rp->tx_power_level;

    net_buf_unref(rsp);
}



static void connected(struct bt_conn *conn, uint8_t err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (err) {
		printk("Failed to connect to %s (%u)\n", addr, err);
		return;
	}

	printk("Connected %s\n", addr);

	if (bt_conn_set_security(conn, BT_SECURITY_L4)) {
		printk("Failed to set security\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Disconnected from %s (reason 0x%02x)\n", addr, reason);
}

static void identity_resolved(struct bt_conn *conn, const bt_addr_le_t *rpa,
			      const bt_addr_le_t *identity)
{
	char addr_identity[BT_ADDR_LE_STR_LEN];
	char addr_rpa[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(identity, addr_identity, sizeof(addr_identity));
	bt_addr_le_to_str(rpa, addr_rpa, sizeof(addr_rpa));

	printk("Identity resolved %s -> %s\n", addr_rpa, addr_identity);
}

static void security_changed(struct bt_conn *conn, bt_security_t level,
			     enum bt_security_err err)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	if (!err) {
		printk("Security changed: %s level %u\n", addr, level);
	} else {
		printk("Security failed: %s level %u err %d\n", addr, level,
		       err);
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.identity_resolved = identity_resolved,
	.security_changed = security_changed,
};

static void auth_passkey_display(struct bt_conn *conn, unsigned int passkey)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Passkey for %s: %06u\n", addr, passkey);
}

static void auth_cancel(struct bt_conn *conn)
{
	char addr[BT_ADDR_LE_STR_LEN];

	bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));

	printk("Pairing cancelled: %s\n", addr);
}

static void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("Pairing Complete\n");
}

static void pairing_failed(struct bt_conn *conn, enum bt_security_err reason)
{
	printk("Pairing Failed (%d). Disconnecting.\n", reason);
	bt_conn_disconnect(conn, BT_HCI_ERR_AUTH_FAIL);
}

static struct bt_conn_auth_cb auth_cb_display = {
	.passkey_display = auth_passkey_display,
	.passkey_entry = NULL,
	.cancel = auth_cancel,
	.pairing_complete = pairing_complete,
	.pairing_failed = pairing_failed,
};

static void bt_ready(int err)
{
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }

    printk("Bluetooth initialized\n");

    /* Start advertising */
    err = bt_le_adv_start(param, ad, ARRAY_SIZE(ad),
                          NULL, 0);
    if (err)
    {
        printk("Advertising failed to start (err %d)\n", err);
        return;
    }

    printk("Dynamic Tx power Beacon started\n");
}

static void hrs_notify(void)
{
    static uint8_t heartrate = 90U;

    /* Heartrate measurements simulation */
    heartrate++;
    if (heartrate == 160U)
    {
        heartrate = 90U;
    }

    bt_hrs_notify(heartrate);
}

void modulate_tx_power(void *p1, void *p2, void *p3)
{
    int8_t txp_get = 0;
    uint8_t idx = 0;

    while (1)
    {
        if (!default_conn)
        {
            gpio_pin_set(dev, PIN, 0);
            printk("Set Tx power level to %d\n", txpower[idx]);
            set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
                         0, txpower[idx]);

            k_sleep(K_SECONDS(5));

            printk("Get Tx power level -> ");
            get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV,
                         0, &txp_get);
            printk("TXP = %d\n", txp_get);

            idx = (idx + 1) % DEVICE_BEACON_TXPOWER_NUM;
        }
        else
        {
            int8_t rssi = 0xFF;
            int8_t txp_adaptive;

            idx = 0;

            read_conn_rssi(default_conn_handle, &rssi);
            printk("Connected (%d) - RSSI = %d\n",
                   default_conn_handle, rssi);
            if (rssi > -40)
            {
                gpio_pin_set(dev, PIN, 1);
            }
            else if (rssi > -70)
            {
                gpio_pin_set(dev, PIN, 0);
                txp_adaptive = -20;
            }
            else if (rssi > -90)
            {
                gpio_pin_set(dev, PIN, 0);
                txp_adaptive = -12;
            }
            else
            {
                gpio_pin_set(dev, PIN, 0);
                txp_adaptive = -4;
            }
            printk("Adaptive Tx power selected = %d\n",
                   txp_adaptive);
            set_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                         default_conn_handle, txp_adaptive);
            get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_CONN,
                         default_conn_handle, &txp_get);
            printk("Connection (%d) TXP = %d\n",
                   default_conn_handle, txp_get);

            k_sleep(K_SECONDS(1));
        }
    }
}

int main(void)
{
    int8_t txp_get = 0xFF;
    int err;
    int ret;

    dev = device_get_binding(LED0);
    if (dev == NULL)
    {
        return 0;
    }

    ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
    if (ret < 0)
    {
        return 0;
    }
    gpio_pin_set(dev, PIN, 0);
    default_conn = NULL;
    printk("Starting Dynamic Tx Power Beacon Demo\n");

    /* Initialize the Bluetooth Subsystem */
    err = bt_enable(bt_ready);
    if (err)
    {
        printk("Bluetooth init failed (err %d)\n", err);
    }
    bt_conn_auth_cb_register(&auth_cb_display);
    if (err)
    {
        printk("Bluetooth passkey set failed: (err %d)\n", err);
    }
    printk("Get Tx power level ->");
    get_tx_power(BT_HCI_VS_LL_HANDLE_TYPE_ADV, 0, &txp_get);
    printk("-> default TXP = %d\n", txp_get);

    /* Wait for 5 seconds to give a chance users/testers
     * to check that default Tx power is indeed the one
     * selected in Kconfig.
     */
    k_sleep(K_SECONDS(5));

    k_thread_create(&pwr_thread_data, pwr_thread_stack,
                    K_THREAD_STACK_SIZEOF(pwr_thread_stack),
                    modulate_tx_power, NULL, NULL, NULL,
                    K_PRIO_COOP(10),
                    0, K_NO_WAIT);
    k_thread_name_set(&pwr_thread_data, "DYN TX");

    while (1)
    {

        hrs_notify();
        k_sleep(K_SECONDS(2));
    }
    return 0;
}
