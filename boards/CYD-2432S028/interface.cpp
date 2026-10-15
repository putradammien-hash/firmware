#include "core/bus_HAL.h"
#include "core/powerSave.h"
#include "core/utils.h"
#include <Arduino.h>
#include <interface.h>

#if defined(HAS_CAPACITIVE_TOUCH)
#if defined(TOUCH_GT911_I2C)
#include "TouchDrvGT911.hpp"
TouchDrvGT911 touch;
struct TouchPointPro {
    int16_t x = 0;
    int16_t y = 0;
};
#else
#include "CYD28_TouchscreenC.h"
#define CYD28_DISPLAY_HOR_RES_MAX 240
#define CYD28_DISPLAY_VER_RES_MAX 320
CYD28_TouchC touch(CYD28_DISPLAY_HOR_RES_MAX, CYD28_DISPLAY_VER_RES_MAX);
#endif
#elif defined(USE_TFT_eSPI_TOUCH)
#define XPT2046_CS TOUCH_CS
#else
#include "CYD28_TouchscreenR.h"
#define CYD28_DISPLAY_HOR_RES_MAX 320
#define CYD28_DISPLAY_VER_RES_MAX 240
CYD28_TouchR touch(CYD28_DISPLAY_HOR_RES_MAX, CYD28_DISPLAY_VER_RES_MAX);
#if defined(TOUCH_XPT2046_SPI)
#define XPT2046_CS XPT2046_SPI_CONFIG_CS_GPIO_NUM
#else
#define XPT2046_CS 33
#endif
#endif

/***************************************************************************************
** Kalibrasi touch resistif (XPT2046) + penyimpanan NVS
** Berlaku untuk env resistif yang memakai CYD28_TouchR (CYD-2432S028, CYD-2USB, dst).
** - Boot pertama (belum ada data di NVS): kalibrasi otomatis 4 titik, lalu disimpan.
** - Boot berikutnya: data dibaca dari NVS, tidak perlu kalibrasi lagi.
** - Kalibrasi ulang: tahan layar (sentuh terus) saat device dinyalakan / reset.
***************************************************************************************/
#if !defined(HAS_CAPACITIVE_TOUCH) && !defined(USE_TFT_eSPI_TOUCH)
#define CYD_RES_TOUCH_NVS_CAL 1
#include <Preferences.h>

#define TCAL_NAMESPACE "touchcal"
#define TCAL_KEY "cal"
#define TCAL_MAGIC 0xCA11
#define TCAL_HOLD_MS 600 // lama menahan layar saat boot untuk memicu kalibrasi ulang

struct TouchCalData {
    uint16_t magic;
    int16_t xmin, xmax, ymin, ymax;
    uint16_t chk;
};

static uint16_t touchCalChecksum(const TouchCalData &d) {
    uint16_t c = 0xA5A5;
    c ^= d.magic;
    c += (uint16_t)d.xmin;
    c ^= (uint16_t)d.xmax;
    c += (uint16_t)d.ymin;
    c ^= (uint16_t)d.ymax;
    return c;
}

static bool touchCalSane(int16_t xmin, int16_t xmax, int16_t ymin, int16_t ymax) {
    int xr = xmax - xmin, yr = ymax - ymin;
    return xr >= 2000 && xr <= 4500 && yr >= 2000 && yr <= 4500 && xmin > -1500 && xmin < 2000 &&
           ymin > -1500 && ymin < 2000;
}

static bool touchCalLoad(TouchCalData &d) {
    Preferences prefs;
    if (!prefs.begin(TCAL_NAMESPACE, true)) return false; // read-only
    size_t len = prefs.getBytesLength(TCAL_KEY);
    bool ok = false;
    if (len == sizeof(TouchCalData)) {
        prefs.getBytes(TCAL_KEY, &d, sizeof(TouchCalData));
        ok = d.magic == TCAL_MAGIC && d.chk == touchCalChecksum(d) &&
             touchCalSane(d.xmin, d.xmax, d.ymin, d.ymax);
    }
    prefs.end();
    return ok;
}

static bool touchCalSave(int16_t xmin, int16_t xmax, int16_t ymin, int16_t ymax) {
    TouchCalData d;
    d.magic = TCAL_MAGIC;
    d.xmin = xmin;
    d.xmax = xmax;
    d.ymin = ymin;
    d.ymax = ymax;
    d.chk = touchCalChecksum(d);

    Preferences prefs;
    if (!prefs.begin(TCAL_NAMESPACE, false)) return false;
    size_t w = prefs.putBytes(TCAL_KEY, &d, sizeof(d));
    prefs.end();
    return w == sizeof(d);
}

// true jika layar ditahan terus-menerus selama TCAL_HOLD_MS (return cepat jika tidak disentuh)
static bool touchHeldAtBoot() {
    uint32_t t0 = millis();
    while (millis() - t0 < TCAL_HOLD_MS) {
        if (!touch.touched()) return false;
        delay(30);
    }
    return true;
}

static void drawCalTarget(int x, int y, uint16_t color) {
    tft.drawCircle(x, y, 10, color);
    tft.drawFastHLine(x - 16, y, 33, color);
    tft.drawFastVLine(x, y - 16, 33, color);
    tft.fillCircle(x, y, 2, color);
}

// Tunggu sentuhan, ambil rata-rata nilai RAW. false jika timeout / sentuhan terlalu singkat.
static bool captureRawPoint(int16_t &rx, int16_t &ry) {
    uint32_t t0 = millis();
    while (!touch.touched()) {
        if (millis() - t0 > 20000) return false;
        delay(10);
    }
    int32_t sx = 0, sy = 0;
    int n = 0;
    for (int i = 0; i < 14; i++) {
        delay(20);
        if (!touch.touched()) break;
        CYD28_TS_Point p = touch.getPointRaw();
        if (i < 4) continue; // abaikan sampel awal (belum stabil)
        sx += p.x;
        sy += p.y;
        n++;
    }
    // tunggu jari dilepas
    t0 = millis();
    int released = 0;
    while (released < 4 && millis() - t0 < 3000) {
        released = touch.touched() ? 0 : released + 1;
        delay(20);
    }
    delay(250);
    if (n < 5) return false;
    rx = sx / n;
    ry = sy / n;
    return true;
}

static bool runTouchCalibration(int16_t &xmin, int16_t &xmax, int16_t &ymin, int16_t &ymax) {
    const int W = CYD28_DISPLAY_HOR_RES_MAX; // 320
    const int H = CYD28_DISPLAY_VER_RES_MAX; // 240
    const int M = 25;
    const int px[4] = {M, W - M, M, W - M};
    const int py[4] = {M, M, H - M, H - M};

    // Kalibrasi dilakukan di rotasi 1 (landscape), di mana koordinat library == koordinat layar.
    // Rotasi pengguna dipulihkan setelah selesai.
    tft.setRotation(1);
    tft.setRotation(1);

    bool success = false;
    for (int attempt = 0; attempt < 3 && !success; attempt++) {
        int16_t rawX[4], rawY[4];
        bool aborted = false;

        for (int i = 0; i < 4 && !aborted; i++) {
            bool got = false;
            for (int tries = 0; tries < 3 && !got; tries++) {
                tft.fillScreen(TFT_BLACK);
                tft.setTextColor(TFT_WHITE, TFT_BLACK);
                tft.setTextSize(2);
                tft.drawCentreString("Kalibrasi Layar", W / 2, 85, 1);
                tft.setTextSize(1);
                char msg[32];
                snprintf(msg, sizeof(msg), "Sentuh titik target (%d/4)", i + 1);
                tft.drawCentreString(msg, W / 2, 115, 1);
                tft.drawCentreString("Gunakan stylus / ujung jari", W / 2, 130, 1);
                drawCalTarget(px[i], py[i], TFT_RED);
                got = captureRawPoint(rawX[i], rawY[i]);
            }
            if (!got) aborted = true;
        }
        if (aborted) break; // timeout: jangan simpan, pakai nilai default

        // Dua titik di kolom/baris yang sama harus punya nilai raw yang berdekatan
        bool consistent = abs(rawX[0] - rawX[2]) < 400 && abs(rawX[1] - rawX[3]) < 400 &&
                          abs(rawY[0] - rawY[1]) < 400 && abs(rawY[2] - rawY[3]) < 400;

        float lx = (rawX[0] + rawX[2]) / 2.0f, rx = (rawX[1] + rawX[3]) / 2.0f;
        float ty = (rawY[0] + rawY[1]) / 2.0f, by = (rawY[2] + rawY[3]) / 2.0f;
        float kx = (rx - lx) / (float)(px[1] - px[0]); // raw per piksel
        float ky = (by - ty) / (float)(py[2] - py[0]);

        int16_t nxmin = (int16_t)roundf(lx - px[0] * kx);
        int16_t nxmax = (int16_t)roundf(lx - px[0] * kx + W * kx);
        int16_t nymin = (int16_t)roundf(ty - py[0] * ky);
        int16_t nymax = (int16_t)roundf(ty - py[0] * ky + H * ky);

        Serial.printf("Touch cal raw: (%d,%d) (%d,%d) (%d,%d) (%d,%d)\n", rawX[0], rawY[0], rawX[1],
                      rawY[1], rawX[2], rawY[2], rawX[3], rawY[3]);

        if (consistent && touchCalSane(nxmin, nxmax, nymin, nymax)) {
            xmin = nxmin;
            xmax = nxmax;
            ymin = nymin;
            ymax = nymax;
            success = true;
        } else {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.setTextSize(2);
            tft.drawCentreString("Gagal, ulangi", W / 2, 105, 1);
            delay(1200);
        }
    }

    if (success) {
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(2);
        tft.drawCentreString("Kalibrasi OK", W / 2, 105, 1);
        delay(700);
    }

    tft.setRotation(bruceConfigPins.rotation);
    tft.setRotation(bruceConfigPins.rotation);
    tft.fillScreen(bruceConfig.bgColor);
    return success;
}

// Dipanggil dari _post_setup_gpio() setelah TFT & backlight siap.
static void touchCalibrationInit() {
    TouchCalData d;
    bool haveSaved = touchCalLoad(d);

    if (haveSaved && !touchHeldAtBoot()) {
        touch.setCalibration(d.xmin, d.xmax, d.ymin, d.ymax);
        Serial.printf("Touch cal dari NVS: %d,%d,%d,%d\n", d.xmin, d.xmax, d.ymin, d.ymax);
        return;
    }

    int16_t xmin, xmax, ymin, ymax;
    if (runTouchCalibration(xmin, xmax, ymin, ymax)) {
        touch.setCalibration(xmin, xmax, ymin, ymax);
        if (touchCalSave(xmin, xmax, ymin, ymax)) Serial.println("Touch cal tersimpan ke NVS");
        else Serial.println("Touch cal GAGAL disimpan ke NVS");
    } else if (haveSaved) {
        // kalibrasi ulang dibatalkan -> tetap pakai data lama
        touch.setCalibration(d.xmin, d.xmax, d.ymin, d.ymax);
    }
    // else: pakai nilai default dari library
}
#endif // resistive touch NVS calibration

/***************************************************************************************
** Function name: _setup_gpio()
** Location: main.cpp
** Description:   initial setup for the device
***************************************************************************************/
SPIClass touchSPI;
void _setup_gpio() {
#ifndef HAS_CAPACITIVE_TOUCH // Capacitive Touchscreen uses I2C to communicate
    pinMode(XPT2046_CS, OUTPUT);
    digitalWrite(XPT2046_CS, HIGH);
#endif

#if defined(HAS_CAPACITIVE_TOUCH)
    setSysI2CBus(&Wire1);
#if defined(TOUCH_GT911_I2C)
    bruceConfigPins.sys_i2c.sda = (gpio_num_t)SYS_I2C_SDA;
    bruceConfigPins.sys_i2c.scl = (gpio_num_t)SYS_I2C_SCL;
#else
    bruceConfigPins.sys_i2c.sda = (gpio_num_t)CYD28_TouchC_SDA;
    bruceConfigPins.sys_i2c.scl = (gpio_num_t)CYD28_TouchC_SCL;
#endif
#endif

#if defined(TOUCH_GT911_I2C)
    pinMode(BOARD_TOUCH_INT, INPUT);
    touch.setPins(-1, BOARD_TOUCH_INT);
    if (!touch.begin(Wire1, GT911_SLAVE_ADDRESS_L, SYS_I2C_SDA, SYS_I2C_SCL)) {
        Serial.println("Failed to find GT911 - check your wiring!");
    }
#else
#if !defined(USE_TFT_eSPI_TOUCH) // Use libraries
    if (!touch.begin()) {
        Serial.println("Touch IC not Started");
        log_i("Touch IC not Started");
    } else log_i("Touch IC Started");
#endif
#endif

    bruceConfig.colorInverted = 0;
}

/***************************************************************************************
** Function name: _post_setup_gpio()
** Location: main.cpp
** Description:   second stage gpio setup to make a few functions work
***************************************************************************************/
void _post_setup_gpio() {
#if defined(USE_TFT_eSPI_TOUCH)
    pinMode(TOUCH_CS, OUTPUT);
    uint16_t calData[5];
    File caldata = LittleFS.open("/calData", "r");

    if (!caldata) {
        tft.setRotation(ROTATION);
        tft.calibrateTouch(calData, TFT_WHITE, TFT_BLACK, 10);

        caldata = LittleFS.open("/calData", "w");
        if (caldata) {
            caldata.printf(
                "%d\n%d\n%d\n%d\n%d\n", calData[0], calData[1], calData[2], calData[3], calData[4]
            );
            caldata.close();
        }
    } else {
        Serial.print("\ntft Calibration data: ");
        for (int i = 0; i < 5; i++) {
            String line = caldata.readStringUntil('\n');
            calData[i] = line.toInt();
            Serial.printf("%d, ", calData[i]);
        }
        Serial.println();
        caldata.close();
    }
    tft.setTouch(calData);
#endif

    // Brightness control must be initialized after tft in this case @Pirata
    pinMode(TFT_BL, OUTPUT);
    ledcAttach(TFT_BL, TFT_BRIGHT_FREQ, TFT_BRIGHT_Bits);
    ledcWrite(TFT_BL, 255);

    // Force sync color inversion to prevent bruceConf.json from overriding
    // the value set in _setup_gpio(). For CYD variants with TFT_INVERSION_ON,
    // the init() sequence sends INVON; we send INVOFF here to ensure normal colors.
#ifdef TFT_INVERSION_ON
    bruceConfig.colorInverted = 0;
    tft.invertDisplay(0);
#else
    bruceConfig.colorInverted = 1;
    tft.invertDisplay(1);
#endif

#if defined(CYD_RES_TOUCH_NVS_CAL)
    // Kalibrasi touch: dibaca dari NVS, atau dilakukan sekali jika belum ada / ditahan saat boot
    touchCalibrationInit();
#endif

    bruceConfigPins.gps_bus.rx = (gpio_num_t)GPS_SERIAL_RX;
    bruceConfigPins.gps_bus.tx = (gpio_num_t)GPS_SERIAL_TX;
    bruceConfigPins.gpsBaudrate = 9600;

    bool pinsChanged = false;
    if (bruceConfigPins.rfTx != 22) {
        bruceConfigPins.rfTx = 22;
        pinsChanged = true;
    }
    if (bruceConfigPins.rfRx != 27) {
        bruceConfigPins.rfRx = 27;
        pinsChanged = true;
    }
    if (bruceConfigPins.irTx != 22) {
        bruceConfigPins.irTx = 22;
        pinsChanged = true;
    }
    if (bruceConfigPins.irRx != 27) {
        bruceConfigPins.irRx = 27;
        pinsChanged = true;
    }
    if (pinsChanged) bruceConfigPins.saveFile();
}

/*********************************************************************
** Function: setBrightness
** location: settings.cpp
** set brightness value
**********************************************************************/
void _setBrightness(uint8_t brightval) {
    int dutyCycle;
    if (brightval == 100) dutyCycle = 255;
    else if (brightval == 75) dutyCycle = 130;
    else if (brightval == 50) dutyCycle = 70;
    else if (brightval == 25) dutyCycle = 20;
    else if (brightval == 0) dutyCycle = 0;
    else dutyCycle = ((brightval * 255) / 100);

    // log_i("dutyCycle for bright 0-255: %d", dutyCycle);
    ledcWrite(TFT_BL, dutyCycle);
}

/*********************************************************************
** Function: InputHandler
** Handles the variables PrevPress, NextPress, SelPress, AnyKeyPress and EscPress
**********************************************************************/
void InputHandler(void) {
    static long d_tmp = 0;
    if (millis() - d_tmp > 200 || LongPress) {
        // I know R3CK.. I Should NOT nest if statements..
        // but it is needed to not keep SPI bus used without need, it save resources
#if defined(USE_TFT_eSPI_TOUCH)
        TouchPoint t;
        checkPowerSaveTime();
        bool _IH_touched = tft.getTouch(&t.x, &t.y);
        if (_IH_touched) {
            NextPress = false;
            PrevPress = false;
            UpPress = false;
            DownPress = false;
            SelPress = false;
            EscPress = false;
            AnyKeyPress = false;
            NextPagePress = false;
            PrevPagePress = false;
            touchPoint.pressed = false;
            _IH_touched = false;
#elif defined(TOUCH_GT911_I2C)
        static unsigned long tm = millis();
        TouchPointPro t;
        uint8_t touched = 0;
        uint8_t rot = 5;

        if (rot != bruceConfigPins.rotation) {
            if (bruceConfigPins.rotation == 1) {
                touch.setMaxCoordinates(TFT_HEIGHT, TFT_WIDTH);
                touch.setSwapXY(true);
                touch.setMirrorXY(false, true);
            }
            if (bruceConfigPins.rotation == 3) {
                touch.setMaxCoordinates(TFT_HEIGHT, TFT_WIDTH);
                touch.setSwapXY(true);
                touch.setMirrorXY(true, false);
            }
            if (bruceConfigPins.rotation == 0) {
                touch.setMaxCoordinates(TFT_WIDTH, TFT_HEIGHT);
                touch.setSwapXY(false);
                touch.setMirrorXY(false, false);
            }
            if (bruceConfigPins.rotation == 2) {
                touch.setMaxCoordinates(TFT_WIDTH, TFT_HEIGHT);
                touch.setSwapXY(false);
                touch.setMirrorXY(true, true);
            }
            rot = bruceConfigPins.rotation;
        }
        // Track touch state to prevent double events on press/release
        static bool lastTouchState = false;
        static unsigned long lastTouchTime = 0;

        touched = touch.getPoint(&t.x, &t.y);
        bool currentTouchState = touched > 0;

        // Only process new touch presses with debouncing
        if (currentTouchState && !lastTouchState && (millis() - lastTouchTime) > 100) {
            // This is a genuine new touch press
            lastTouchTime = millis();
        } else if (!currentTouchState || lastTouchState) {
            // Touch release or continuing touch - ignore
            touched = 0;
        }
        lastTouchState = currentTouchState;
        if (((millis() - tm) > 190 || LongPress) && touched) {
            tm = millis();
#else
        if (touch.touched()) {
            auto t = touch.getPointScaled();
#endif
#if !defined(TOUCH_GT911_I2C)
            // Serial.printf("\nRAW: Touch Pressed on x=%d, y=%d",t.x, t.y);
            if (bruceConfigPins.rotation == 3) {
                t.y = (tftHeight + 20) - t.y;
                t.x = tftWidth - t.x;
            }
            if (bruceConfigPins.rotation == 0) {
                int tmp = t.x;
                t.x = tftWidth - t.y;
                t.y = tmp;
            }
            if (bruceConfigPins.rotation == 2) {
                int tmp = t.x;
                t.x = t.y;
                t.y = (tftHeight + 20) - tmp;
            }
#endif
            // Serial.printf("\nROT: Touch Pressed on x=%d, y=%d\n", t.x, t.y);

            if (!wakeUpScreen()) AnyKeyPress = true;
            else goto END;

            // Touch point global variable
            touchPoint.x = t.x;
            touchPoint.y = t.y;
            touchPoint.pressed = true;
            touchHeatMap(touchPoint);
        END:
            d_tmp = millis();
        }
    }
}

/*********************************************************************
** Function: powerOff
** location: mykeyboard.cpp
** Turns off the device (or try to)
**********************************************************************/
void powerOff() {
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_0, LOW);
    esp_deep_sleep_start();
}

/*********************************************************************
** Function: checkReboot
** location: mykeyboard.cpp
** Btn logic to turn off the device (name is odd btw)
**********************************************************************/
void checkReboot() {}
