from maix import pinmap, time, uart


UART_DEVICE = "/dev/ttyS2"
BAUDRATE = 115200


def main():
    print("UART2 test start")
    print("device:", UART_DEVICE, "baud:", BAUDRATE)

    ret_tx = pinmap.set_pin_function("B0", "UART2_TX")
    ret_rx = pinmap.set_pin_function("B1", "UART2_RX")
    print("pinmap B0 -> UART2_TX:", ret_tx)
    print("pinmap B1 -> UART2_RX:", ret_rx)

    serial = uart.UART(UART_DEVICE, BAUDRATE)
    rx_buf = bytearray()

    last_ping_ms = 0
    last_target_ms = 0
    fake_dx = -80
    step = 4

    while True:
        now = time.ticks_ms()

        if now - last_ping_ms >= 1000:
            last_ping_ms = now
            serial.write(b"PING\n")
            print("TX: PING")

        if now - last_target_ms >= 50:
            last_target_ms = now
            msg = "{},{}\n".format(fake_dx, 0)
            serial.write(msg.encode("ascii"))
            print("TX:", msg.strip())

            fake_dx += step
            if fake_dx > 80 or fake_dx < -80:
                step = -step

        data = serial.read()
        if data:
            raw = bytes(data)
            print("RX_RAW:", raw)

            for b in raw:
                if b == 10:
                    line = bytes(rx_buf).decode("ascii", "ignore").strip()
                    rx_buf = bytearray()
                    if line:
                        print("RX_LINE:", line)
                elif b != 13:
                    if len(rx_buf) < 128:
                        rx_buf.append(b)
                    else:
                        rx_buf = bytearray()

        time.sleep_ms(10)


if __name__ == "__main__":
    main()
