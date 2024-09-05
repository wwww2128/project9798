#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>
#include <termios.h> // UART 통신을 위한 헤더 추가

#define PCF8591_ADDR 0x48
#define PCF8591_INPUT_REG 0x00

#define LCD_ADDR 0x27
#define LCD_CHR 1
#define LCD_CMD 0

#define LCD_BACKLIGHT 0x08  // LCD 백라이트
#define ENABLE 0b00000100  // Enable 비트
#define LINE1 0x80
#define LINE2 0xC0

#define WARNING_WATER_LEVEL 100 // 경고 수위 임계값
#define GAS_WARNING_LEVEL 135   // 경고 가스 임계값
#define WATER_PIN 0 // 버저용 GPIO 핀
#define GAS_PIN 7 //

int i2c_fd; // I2C 파일 디스크립터
int file;   // ADC 파일 디스크립터
int uart0_filestream = -1; // UART 파일 디스크립터

void lcd_byte(int bits, int mode);
void lcd_toggle_enable(int bits);
void lcd_init();            // LCD 초기화
void typeInt(int i);
void lcdLoc(int line);      // 커서 위치 설정
void ClrLcd();              // 디스플레이 초기화
void typeIn(const char *s); // LCD에 문자열 출력
void typeChar(char val);
int readADC(int channel);   // ADC 값 읽기
void uart_init();           // UART 초기화
void uart_send_data(int water_level, int gas_level); // UART로 데이터 전송

int main() {
    char *filename = "/dev/i2c-1";

    // WiringPi 초기화
    if (wiringPiSetup() == -1) {
        perror("WiringPi 설정 실패");
        return 1;
    }

    // GPIO 핀 모드 설정
    pinMode(WATER_PIN, OUTPUT);
    digitalWrite(WATER_PIN, LOW);
    pinMode(GAS_PIN, OUTPUT); 
    digitalWrite(GAS_PIN, LOW);

    // LCD용 I2C 초기화
    i2c_fd = wiringPiI2CSetup(LCD_ADDR);
    if (i2c_fd == -1) {
        perror("I2C LCD 초기화 실패");
        return 1;
    }

    lcd_init();

    // ADC용 I2C 초기화
    if ((file = open(filename, O_RDWR)) < 0) {
        perror("I2C 버스 열기 실패");
        return 1;
    }

    if (ioctl(file, I2C_SLAVE, PCF8591_ADDR) < 0) {
        perror("버스 접근 및 슬레이브와 통신 실패");
        close(file);
        return 1;
    }

    // UART 초기화
    uart_init();

    int last_adc_value_water = -1; // 이전 수위 ADC 값
    int last_adc_value_gas = -1;   // 이전 가스 ADC 값

    while (1) {
        int adc_value_water = readADC(1); // 수위 센서 값 읽기
        int adc_value_gas = readADC(0);   // 가스 센서 값 읽기

        // LCD 업데이트 및 경고 처리
        if (adc_value_water != last_adc_value_water || adc_value_gas != last_adc_value_gas) {
            ClrLcd();

            lcdLoc(LINE1);
            if (adc_value_water >= WARNING_WATER_LEVEL) {
                typeIn("WATER: WARNING");
                digitalWrite(WATER_PIN, HIGH);
            } else {
                char buffer_str[16];
                snprintf(buffer_str, sizeof(buffer_str), "WATER: %d", adc_value_water);
                typeIn(buffer_str);
                digitalWrite(WATER_PIN, LOW);
            }

            lcdLoc(LINE2);
            if (adc_value_gas >= GAS_WARNING_LEVEL) {
                typeIn("GAS: WARNING");
                digitalWrite(GAS_PIN, HIGH);
            } else {
                char buffer_str[16];
                snprintf(buffer_str, sizeof(buffer_str), "GAS: %d", adc_value_gas);
                typeIn(buffer_str);
                digitalWrite(GAS_PIN, LOW);
            }

            // UART로 데이터 전송
            uart_send_data(adc_value_water, adc_value_gas);

            last_adc_value_water = adc_value_water;
            last_adc_value_gas = adc_value_gas;
        }

        sleep(1); // 1초 대기
    }

    close(file);
    close(uart0_filestream);
    return 0;
}

void uart_init() {
    // UART 포트 열기
    uart0_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);

    if (uart0_filestream == -1) {
        printf("Error - UART 열기 실패. 다른 애플리케이션이 사용 중인지 확인하세요.\n");
        exit(-1);
    }

    // UART 설정
    struct termios options;
    tcgetattr(uart0_filestream, &options);
    options.c_cflag = B9600 | CS8 | CLOCAL | CREAD;
    options.c_iflag = IGNPAR;
    options.c_oflag = 0;
    options.c_lflag = 0;
    tcflush(uart0_filestream, TCIFLUSH);
    tcsetattr(uart0_filestream, TCSANOW, &options);
}

void uart_send_data(int water_level, int gas_level) {
    if (uart0_filestream != -1) {
        // 데이터를 세미콜론으로 구분된 문자열로 포맷
        char data[20];
        snprintf(data, sizeof(data), "%d;%d;", water_level, gas_level);

        // 데이터 전송
        int count = write(uart0_filestream, &data[0], strlen(data));
        if (count < 0) {
            printf("UART 전송 오류\n");
        } else {
            printf("Sent: %s\n", data);
        }
    }
}

void lcd_byte(int bits, int mode) {
    int bits_high;
    int bits_low;
    bits_high = mode | (bits & 0xF0) | LCD_BACKLIGHT; 
    bits_low = mode | ((bits << 4) & 0xF0) | LCD_BACKLIGHT;

    wiringPiI2CWriteReg8(i2c_fd, 0x00, bits_high);
    lcd_toggle_enable(bits_high);

    wiringPiI2CWriteReg8(i2c_fd, 0x00, bits_low);
    lcd_toggle_enable(bits_low);
}

void lcd_toggle_enable(int bits) {
    delayMicroseconds(500);
    wiringPiI2CWriteReg8(i2c_fd, 0x00, (bits | ENABLE));
    delayMicroseconds(500);
    wiringPiI2CWriteReg8(i2c_fd, 0x00, (bits & ~ENABLE));
    delayMicroseconds(500);
}

void lcd_init() {
    lcd_byte(0x33, LCD_CMD);
    lcd_byte(0x32, LCD_CMD);
    lcd_byte(0x06, LCD_CMD);
    lcd_byte(0x0C, LCD_CMD);
    lcd_byte(0x28, LCD_CMD);
    lcd_byte(0x01, LCD_CMD);
    delayMicroseconds(500);
}

void typeInt(int i) {
    char array1[20];
    sprintf(array1, "%d", i);
    typeIn(array1);
}

void ClrLcd() {
    lcd_byte(0x01, LCD_CMD);
    lcd_byte(0x02, LCD_CMD);
}

void lcdLoc(int line) {
    lcd_byte(line, LCD_CMD);
}

void typeChar(char val) {
    lcd_byte(val, LCD_CHR);
}

void typeIn(const char *s) {
    while (*s) lcd_byte(*(s++), LCD_CHR);
}

int readADC(int channel) {
    unsigned char buffer[1];
    buffer[0] = PCF8591_INPUT_REG + channel;
    
    if (write(file, buffer, 1) != 1) {
        perror("I2C 버스 쓰기 실패");
        return -1;
    }

    if (read(file, buffer, 1) != 1) {
        perror("I2C 버스 읽기 실패");
        return -1;
    }

    return buffer[0];
}
