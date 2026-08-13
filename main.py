try:
    from machine import Pin, SPI
    MACHINE_AVAILABLE = True
except Exception:
    MACHINE_AVAILABLE = False

try:
    import uos as os
except ImportError:  # pragma: no cover - host Python fallback
    import os

try:
    from sdcard import SDCard
except Exception:
    SDCard = None


MOUNT_POINT = "/sd"
SD_FILE_PATH = "logs.txt"
file_path = "{}/{}".format(MOUNT_POINT, SD_FILE_PATH)

# Raspberry Pi Pico / Pico W defaults for an external SPI SD card reader.
SPI_ID = 0
SCK_PIN = 18
MOSI_PIN = 19
MISO_PIN = 16
CS_PIN = 17


def mount_sd_card():
    if not MACHINE_AVAILABLE or SDCard is None:
        raise RuntimeError("SD card interface not available on this platform")
    spi = SPI(
        SPI_ID,
        baudrate=10_000_000,
        polarity=0,
        phase=0,
        sck=Pin(SCK_PIN),
        mosi=Pin(MOSI_PIN),
        miso=Pin(MISO_PIN),
    )
    sd = SDCard(spi, Pin(CS_PIN))
    os.mount(sd, MOUNT_POINT)
    return sd


def create_file_on_sd(filename, content):
    mount_point = MOUNT_POINT
    path = "{}/{}".format(mount_point.rstrip('/'), filename)
    # Ensure directory exists for host-side testing (MicroPython: uos has no path/makedirs)
    dir_path = mount_point
    try:
        os.stat(dir_path)
    except Exception:
        try:
            os.mkdir(dir_path)
        except Exception:
            pass
    with open(path, "w") as handle:
        handle.write(content)
    print("Created file:", path)
    return path


def list_files(path):
    for entry in os.listdir(path):
        print(entry)


def read_file(path):
    if SD_FILE_PATH in os.listdir(MOUNT_POINT):
        with open(path, "r") as handle:
            print(handle.read())
    else:
        print("\nCreate {} on the SD card to read it here.".format(SD_FILE_PATH))



def main():
    sd = None
    try:
        sd = mount_sd_card()
        create_file_on_sd('test.txt', "Hello, SD card!")
    except Exception as exc:
        # On non-hardware platforms, mounting will likely fail — warn and exit
        print("SD card not available:", exc)
    finally:
        # Attempt to unmount the SD card if it was mounted
        try:
            if sd is not None and hasattr(sd, "deinit"):
                try:
                    sd.deinit()
                except Exception:
                    pass
            if hasattr(os, "umount"):
                try:
                    os.umount(MOUNT_POINT)
                    print("Unmounted SD card at", MOUNT_POINT)
                except Exception as e:
                    print("Failed to unmount SD card:", e)
            else:
                print("os.umount not available on this platform; skipping unmount")
        except Exception as e:
            print("Unmount cleanup failed:", e)


if __name__ == '__main__':
    main()