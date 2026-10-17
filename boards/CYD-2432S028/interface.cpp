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
** Mengikuti contoh bawaan TFT_eSPI (Generic/Touch_calibrate -> tft.calibrateTouch()):
**  - 4 sudut layar (panah), 8 sampel per sudut, rotasi aktif
**  - hasil berupa calData[5] = {x0, x_range, y0, y_range, flags}
**    flags: bit0 = sumbu X/Y sentuhan tertukar, bit1 = invert X, bit2 = invert Y
**  - konversi raw -> koordinat layar sama dengan TFT_eSPI::convertRawXY()
** Beda dengan contoh aslinya: data disimpan di NVS (bukan SPIFFS), satu slot per rotasi,
** dan pembacaan raw memakai library CYD28_TouchR (touch ada di bus SPI terpisah).
** - Rotasi belum punya data di NVS -> kalibrasi otomatis lalu disimpan.
** - Kalibrasi ulang: tahan layar (sentuh terus) saat device dinyalakan / reset.
***************************************************************************************/
#if !defined(HAS_CAPACITIVE_TOUCH) && !defined(USE_TFT_eSPI_TOUCH)
#define CYD_RES_TOUCH_NVS_CAL 1
#include <Preferences.h>

#define TCAL_NAMESPACE "touchcal"
#define TCAL_MAGIC 0xCA12
#define TCAL_ARROW 15    // ukuran panah sudut (sama dengan contoh TFT_eSPI)
#define TCAL_HOLD_MS 600 // lama menahan layar saat boot untuk memicu kalibrasi ulang

struct TouchCalSlot {
    uint16_t magic;
    uint16_t d[5];
    uint16_t chk;
};

static uint16_t touchCalData[5];  // calData aktif (format TFT_eSPI)
static bool touchCalActive = false;
static uint8_t touchCalRot = 255; // rotasi yang calData-nya sedang dipakai

static uint16_t touchCalChecksum(const TouchCalSlot &s) {
    uint16_t c = 0xA5A5 ^ s.magic;
    for (int i = 0; i < 5; i++) c = (uint16_t)((c << 3) | (c >> 13)) ^ s.d[i];
    return c;
}

static bool touchCalSane(const uint16_t *d) {
    return d[1] >= 1500 && d[1] <= 4500 && d[3] >= 1500 && d[3] <= 4500 && d[0] < 2500 && d[2] < 2500 &&
           d[4] <= 7;
}

static void touchCalKey(uint8_t rot, char *key) { snprintf(key, 8, "r%u", (unsigned)(rot & 3)); }

static bool touchCalLoad(uint8_t rot, uint16_t *d) {
    char key[8];
    touchCalKey(rot, key);
    Preferences prefs;
    if (!prefs.begin(TCAL_NAMESPACE, true)) return false; // read-only
    TouchCalSlot s;
    bool ok = false;
    if (prefs.getBytesLength(key) == sizeof(s)) {
        prefs.getBytes(key, &s, sizeof(s));
        ok = s.magic == TCAL_MAGIC && s.chk == touchCalChecksum(s) && touchCalSane(s.d);
        if (ok) memcpy(d, s.d, sizeof(s.d));
    }
    prefs.end();
    return ok;
}

static bool touchCalSave(uint8_t rot, const uint16_t *d) {
    char key[8];
    touchCalKey(rot, key);
    TouchCalSlot s;
    s.magic = TCAL_MAGIC;
    memcpy(s.d, d, sizeof(s.d));
    s.chk = touchCalChecksum(s);
    Preferences prefs;
    if (!prefs.begin(TCAL_NAMESPACE, false)) return false;
    size_t w = prefs.putBytes(key, &s, sizeof(s));
    prefs.end();
    return w == sizeof(s);
}

// Port dari TFT_eSPI::convertRawXY(): raw XPT2046 -> koordinat layar (rotasi aktif)
static void touchCalConvert(int16_t rawX, int16_t rawY, int16_t &x, int16_t &y) {
    const int32_t W = tft.width(), H = tft.height();
    const bool rotate = touchCalData[4] & 0x01;
    const bool invX = touchCalData[4] & 0x02;
    const bool invY = touchCalData[4] & 0x04;
    int32_t a = rotate ? rawY : rawX;
    int32_t b = rotate ? rawX : rawY;
    int32_t xx = (a - (int32_t)touchCalData[0]) * W / (int32_t)touchCalData[1];
    int32_t yy = (b - (int32_t)touchCalData[2]) * H / (int32_t)touchCalData[3];
    if (invX) xx = W - xx;
    if (invY) yy = H - yy;
    if (xx < 0) xx = 0;
    if (xx > W - 1) xx = W - 1;
    if (yy < 0) yy = 0;
    if (yy > H - 1) yy = H - 1;
    x = xx;
    y = yy;
}

// Pastikan data kalibrasi yang aktif sesuai rotasi saat ini (rotasi bisa diubah dari menu Settings).
// Rotasi tanpa data -> touchCalActive=false (pakai pemetaan lama) sampai kalibrasi di boot berikutnya.
static void touchCalSyncRotation() {
    uint8_t rot = bruceConfigPins.rotation & 3;
    if (rot == touchCalRot) return;
    touchCalRot = rot;
    touchCalActive = touchCalLoad(rot, touchCalData);
}

// Isi p dengan koordinat layar terkalibrasi. false jika belum ada kalibrasi untuk rotasi ini.
static bool touchCalGetPoint(CYD28_TS_Point &p) {
    touchCalSyncRotation();
    if (!touchCalActive) return false;
    CYD28_TS_Point r = touch.getPointRaw();
    int16_t x, y;
    touchCalConvert(r.x, r.y, x, y);
    p.x = x;
    p.y = y;
    p.z = r.z;
    return true;
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

// Gambar panah di salah satu sudut (sama dengan TFT_eSPI::calibrateTouch)
static void drawCalCorner(uint8_t i, int W, int H, int size, uint16_t fg, uint16_t bg) {
    tft.fillRect(0, 0, size + 1, size + 1, bg);
    tft.fillRect(0, H - size - 1, size + 1, size + 1, bg);
    tft.fillRect(W - size - 1, 0, size + 1, size + 1, bg);
    tft.fillRect(W - size - 1, H - size - 1, size + 1, size + 1, bg);
    switch (i) {
        case 0: // kiri atas
            tft.drawLine(0, 0, 0, size, fg);
            tft.drawLine(0, 0, size, 0, fg);
            tft.drawLine(0, 0, size, size, fg);
            break;
        case 1: // kiri bawah
            tft.drawLine(0, H - size - 1, 0, H - 1, fg);
            tft.drawLine(0, H - 1, size, H - 1, fg);
            tft.drawLine(size, H - size - 1, 0, H - 1, fg);
            break;
        case 2: // kanan atas
            tft.drawLine(W - size - 1, 0, W - 1, 0, fg);
            tft.drawLine(W - size - 1, size, W - 1, 0, fg);
            tft.drawLine(W - 1, size, W - 1, 0, fg);
            break;
        case 3: // kanan bawah
            tft.drawLine(W - size - 1, H - size - 1, W - 1, H - 1, fg);
            tft.drawLine(W - 1, H - 1 - size, W - 1, H - 1, fg);
            tft.drawLine(W - 1 - size, H - 1, W - 1, H - 1, fg);
            break;
    }
}

// Port dari TFT_eSPI::calibrateTouch(): urutan sudut kiri-atas, kiri-bawah, kanan-atas, kanan-bawah.
// false jika timeout (tidak ada sentuhan) atau hasil tidak masuk akal.
static bool touchCalibrate(uint16_t *params) {
    const int W = tft.width(), H = tft.height(), size = TCAL_ARROW;
    const uint16_t fg = TFT_MAGENTA, bg = TFT_BLACK;
    int32_t v[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    for (uint8_t i = 0; i < 4; i++) {
        tft.fillScreen(bg);
        tft.setTextColor(TFT_WHITE, bg);
        tft.setTextSize(1);
        tft.drawCentreString("Kalibrasi layar", W / 2, H / 2 - 20, 1);
        tft.drawCentreString("Sentuh ujung panah", W / 2, H / 2 - 5, 1);
        tft.drawCentreString("(tahan sampai 8 sampel)", W / 2, H / 2 + 10, 1);
        drawCalCorner(i, W, H, size, fg, bg);
        if (i > 0) delay(1000); // beri waktu melepas jari

        uint32_t t0 = millis();
        for (uint8_t j = 0; j < 8;) {
            if (millis() - t0 > 20000) return false; // timeout
            if (!touch.touched()) {
                delay(5);
                continue;
            }
            CYD28_TS_Point r = touch.getPointRaw();
            v[i * 2] += r.x;
            v[i * 2 + 1] += r.y;
            j++;
            delay(15);
        }
        v[i * 2] /= 8;
        v[i * 2 + 1] /= 8;
    }

    bool rotate = false, invX = false, invY = false;
    int32_t x0, x1, y0, y1;
    // Dari sudut 0 ke 1 nilai Y layar berubah. Jika perubahan X raw lebih besar dari Y raw,
    // sumbu sentuhan dan layar tertukar.
    if (abs(v[0] - v[2]) > abs(v[1] - v[3])) {
        rotate = true;
        x0 = (v[1] + v[3]) / 2;
        x1 = (v[5] + v[7]) / 2;
        y0 = (v[0] + v[4]) / 2;
        y1 = (v[2] + v[6]) / 2;
    } else {
        x0 = (v[0] + v[2]) / 2;
        x1 = (v[4] + v[6]) / 2;
        y0 = (v[1] + v[5]) / 2;
        y1 = (v[3] + v[7]) / 2;
    }
    if (x0 > x1) {
        int32_t t = x0;
        x0 = x1;
        x1 = t;
        invX = true;
    }
    if (y0 > y1) {
        int32_t t = y0;
        y0 = y1;
        y1 = t;
        invY = true;
    }
    x1 -= x0;
    y1 -= y0;
    if (x0 == 0) x0 = 1;
    if (x1 == 0) x1 = 1;
    if (y0 == 0) y0 = 1;
    if (y1 == 0) y1 = 1;

    Serial.printf(
        "Touch cal raw: UL(%d,%d) BL(%d,%d) UR(%d,%d) BR(%d,%d)\n", (int)v[0], (int)v[1], (int)v[2],
        (int)v[3], (int)v[4], (int)v[5], (int)v[6], (int)v[7]
    );

    if (x0 < 0 || y0 < 0) return false;
    params[0] = x0;
    params[1] = x1;
    params[2] = y0;
    params[3] = y1;
    params[4] = (rotate ? 1 : 0) | (invX ? 2 : 0) | (invY ? 4 : 0);
    return touchCalSane(params);
}

// Dipanggil dari _post_setup_gpio() setelah TFT & backlight siap.
static void touchCalibrationInit() {
    const uint8_t rot = bruceConfigPins.rotation & 3;
    touchCalRot = rot;
    uint16_t d[5];
    bool haveSaved = touchCalLoad(rot, d);

    if (haveSaved && !touchHeldAtBoot()) {
        memcpy(touchCalData, d, sizeof(d));
        touchCalActive = true;
        Serial.printf(
            "Touch cal (rot %u) dari NVS: %u,%u,%u,%u,%u\n", rot, d[0], d[1], d[2], d[3], d[4]
        );
        return;
    }

    tft.setRotation(bruceConfigPins.rotation);
    tft.setRotation(bruceConfigPins.rotation);

    uint16_t n[5];
    bool ok = false;
    for (int attempt = 0; attempt < 3 && !ok; attempt++) {
        ok = touchCalibrate(n);
        if (!ok) {
            tft.fillScreen(TFT_BLACK);
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.setTextSize(2);
            tft.drawCentreString("Gagal, ulangi", tft.width() / 2, tft.height() / 2 - 8, 1);
            delay(1200);
        }
    }

    if (ok) {
        memcpy(touchCalData, n, sizeof(n));
        touchCalActive = true;
        if (touchCalSave(rot, n)) Serial.println("Touch cal tersimpan ke NVS");
        else Serial.println("Touch cal GAGAL disimpan ke NVS");
        tft.fillScreen(TFT_BLACK);
        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.setTextSize(2);
        tft.drawCentreString("Kalibrasi OK", tft.width() / 2, tft.height() / 2 - 8, 1);
        delay(700);
    } else if (haveSaved) {
        // kalibrasi ulang dibatalkan -> tetap pakai data lama
        memcpy(touchCalData, d, sizeof(d));
        touchCalActive = true;
    }
    // else: touchCalActive=false -> pakai pemetaan bawaan library (nilai default)

    tft.fillScreen(bruceConfig.bgColor);
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
#if defined(CYD_RES_TOUCH_NVS_CAL)
            CYD28_TS_Point t;
            // Terkalibrasi (NVS): koordinat sudah sesuai rotasi aktif, rotasi manual dilewati
            bool touchCalibrated = touchCalGetPoint(t);
            if (!touchCalibrated) t = touch.getPointScaled();
#else
            auto t = touch.getPointScaled();
#endif
#endif
#if !defined(TOUCH_GT911_I2C)
            // Serial.printf("\nRAW: Touch Pressed on x=%d, y=%d",t.x, t.y);
#if defined(CYD_RES_TOUCH_NVS_CAL)
            if (!touchCalibrated) {
#endif
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
#if defined(CYD_RES_TOUCH_NVS_CAL)
            }
#endif
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
