#include "misc.h"
#include "ad7293.h"
#include <Ethernet.h>
#include <math.h>
#include <ctype.h>
#include <RP2040.h>

// Enter a MAC address and IP address for your controller below.
// The IP address will be dependent on your local network.
// gateway and subnet are optional:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 0, 177);
IPAddress myDns(192, 168, 0, 1);
IPAddress gateway(192, 168, 0, 10);
IPAddress subnet(255, 255, 255, 0);


EthernetServer server(2075);
EthernetClient clients[8];
#define ETH_RESET_PIN 20

#define RX_MAX_LEN 15
char rx_buff[RX_MAX_LEN];
#define CMD_MAX_LEN 3
char cmd_buff[CMD_MAX_LEN+1];
char response_str_buff[100];
////////////////////////////////////////////

#define LED0_PIN 25
#define BUTTON1_PIN 8
#define BUTTON2_PIN 7
#define BUTTON3_PIN 6
#define AD_ALERT0 9

#define LED1_PIN 0
#define LED2_PIN 1
#define PSU_EN_PIN 2
#define PSU_PG_PIN 3
#define FAN_CTL_PIN 4
#define PREAMP_CTL_PIN 5

#define TEMP_SENSOR_PIN 26 //ADC0


struct ad7293_dev* ad7293_obj;
uint16_t data_val;

float target_amps = 3.0;
float rs0_volts;
float isense0_amps, isense1_amps;
float Ug0_volts, Ug1_volts;
float Ug0_volts_lim = 0, Ug1_volts_lim = 0; 
uint8_t rs0_alert_high = 0;
uint8_t rs0_alert_low = 0;
uint8_t alert0_state = 0;

uint16_t rsx_alerts;
int ad_monitoring_halt = 1;
int open_loop_mode = 0;
int pa_on_state = 0;
int psu_pg_state = 0;
float temperature_degC = 0;
float temperature1_degC = 0;
float temperature2_degC = 0;
float temperature3_degC = 0;
float temp_degC_Fpwr = 0; 
float temp_degC_Rpwr = 0;
float fwdpwr_dbm = 0; 
float refpwr_dbm = 0;
float refpwr = 0;
float fwdpwr = 0;
float s11_param = 0;

////////////////////////////////////////////
void psu_en(uint8_t en_state){
  if(en_state){
    digitalWrite(PSU_EN_PIN, HIGH);

    while(digitalRead(PSU_PG_PIN) == LOW){
      delay(100); 
      Serial.printf("Enabling the PSU...\n"); 
    }
  }else{
    digitalWrite(PSU_EN_PIN, LOW);

    while(digitalRead(PSU_PG_PIN) == HIGH){
      delay(100); 
      Serial.printf("Disabling the PSU...\n"); 
    }
  }

}

void fan_en(uint8_t en_state){
  if(en_state){
    digitalWrite(FAN_CTL_PIN, HIGH);
  }
  else{
    digitalWrite(FAN_CTL_PIN, LOW);
  }
}

void preamp_en(uint8_t en_state){
  delay(100);

  if(en_state){
    digitalWrite(PREAMP_CTL_PIN, HIGH);
  }
  else{
    digitalWrite(PREAMP_CTL_PIN, LOW);
  }

  delay(100);
}

void rs0_raw_to_voltage(uint16_t raw_in, float* voltage_out){

  *voltage_out = (((float)(raw_in>>4) + 0.5)/4096)*ADC_REF*50;
}

void bi_vout_raw_to_voltage(uint16_t raw_in, float* voltage_out){
  *voltage_out = (((float)(raw_in>>4) + 0.5)/4096)*ADC_REF*8 - 5.0;
}

void isense_raw_to_current(uint16_t raw_in, float* current_out, float r_sense){

  *current_out = (((float)((raw_in>>4) - 2047.5f))/2048)*ADC_REF/(U_SENSE_GAIN*r_sense);
}

void tsense_raw_to_celsius(uint16_t raw_in, float* temp) {
    // Handle the base temperature from D15 (bit 15)
    *temp = (raw_in & (1 << 15)) ? 0.0f : -256.0f;

    // Values for bits D14 (14) to D4 (4) when set
    const float bit_values[] = {
        128.0f,   // D14 (bit 14)
        64.0f,    // D13 (bit 13)
        32.0f,    // D12 (bit 12)
        16.0f,    // D11 (bit 11)
        8.0f,     // D10 (bit 10)
        4.0f,     // D9  (bit 9)
        2.0f,     // D8  (bit 8)
        1.0f,     // D7  (bit 7)
        0.5f,     // D6  (bit 6)
        0.25f,    // D5  (bit 5)
        0.125f    // D4  (bit 4)
    };

    // Check each relevant bit from D14 (14) to D4 (4)
    for (int i = 0; i < 11; i++) {
        int bit_position = 14 - i;
        if (raw_in & (1 << bit_position)) {
            *temp += bit_values[i];
        }
    }
}

void vin_raw_to_celsius(uint16_t raw_in, float* temp) {
    // Extract 12-bit ADC value from the 16-bit register
    // Convert ADC value to voltage (5V reference)
    // Convert voltage to temperature using datasheet formula: T = (V - 1.4) / 0.0048 + 25
    *temp = ((((((raw_in >> 4) + 0.5f) / 4096.0f) * ADC_REF * 2) -1.4f) / 0.0048f) + 25.0f;
}

// Convert 16-bit register (12-bit ADC) to dBm (linear approximation)
void adc_to_dbm_ref(uint16_t raw_in, float* dbm) {
    // Extract 12-bit ADC value from the 16-bit register
    // Convert ADC to voltage (5V reference)
    // Linear approximation: dBm = 38.25 * voltage - 64.41
    *dbm = 38.25f * ((((raw_in >> 4) + 0.5f) / 4096.0f) * ADC_REF * 2) - 64.41f;
}

// Convert 16-bit register (12-bit ADC) to dBm (linear approximation)
void adc_to_dbm_fwd(uint16_t raw_in, float* dbm) {
    // Extract 12-bit ADC value from the 16-bit register
    // Convert ADC to voltage (5V reference)
    // Linear approximation: dBm = 38.25 * voltage - 64.41
    *dbm = 33.24f * ((((raw_in >> 4) + 0.5f) / 4096.0f ) * ADC_REF * 2) - 53.457f;
}

void dbm_to_watts(float dbm, float* pwr) {
    // Convert dBm to Watts using: P(W) = 100 * 10^(dBm/10)
    *pwr = 100.0f * powf(10.0f, dbm / 10.0f);
}

// Calculate return loss S11 parameter
void ref_fwd_s11(float refpwr, float fwdpwr, float* s11){
  *s11 =  10.0f * log10f(fwdpwr / refpwr);
}

uint16_t get_dac_value(float target_amps, float r_sense){
  float u_sense = target_amps*r_sense;
  float dac_vout = u_sense*U_SENSE_GAIN;
  return (uint16_t)(16390*(dac_vout-DAC_OFFSET)/(2*DAC_VREF));
}


void open_loop_enable(uint8_t open_loop_state_in){

  ad_monitoring_halt = 1;
  if(pa_on_state && (open_loop_state_in == 1)){
              
    uint16_t v_out_0, v_out_1; 
    ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT0_MON, &v_out_0);
    bi_vout_raw_to_voltage(v_out_0, &Ug0_volts_lim);
    ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT1_MON, &v_out_1);
    bi_vout_raw_to_voltage(v_out_1, &Ug1_volts_lim);

    Serial.printf("Init Ug0: %0.3f V,  Ug1: %0.3f V\n", Ug0_volts_lim, Ug1_volts_lim);

    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_HL, v_out_0);
    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_LL, v_out_0);

    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_HL, v_out_1);
    ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_LL, v_out_1);
    
    Serial.printf("Open loop mode enabled...\n");           
      
    open_loop_mode = 1;
  }
  else{
      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_HL, 0xfff0);
      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_MON_LL, 0);

      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_HL, 0xfff0);
      ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_MON_LL, 0);
      Serial.printf("Open loop mode disabled...\n");
      open_loop_mode = 0;
  }

      ad_monitoring_halt = 0;
}

int ad7293_init_and_configure(uint8_t pa_on){

  ad_monitoring_halt = 1;
  int ret = -1;

  ad_reset(1);

  ret = ad7293_init(&ad7293_obj, NULL);
  if(ret != 0)
    return ret;

  open_loop_mode = 0;//reset state always off

  delay(2000);

  //internal ADC ref, ALERT0 clamp:
  data_val = (1 << 7)|(1 << 1);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_GENERAL, data_val, data_val); 
  if(ret != 0)
    return ret;
  //RS0+ and BiVout0, BiVeout1 mon voltage background monitoring enable:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RSX_MON_BG_EN, (1 << 8) | (1 << 4) | (1 << 5)); 
  if(ret != 0)
    return ret;
  //isense ch0, ch1 background monitoring enable:
  data_val = (1 << 0)|(1 << 1);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_ISENSE_BG_EN, data_val, data_val); 
  if(ret != 0)
    return ret;

  //tsense int, d0, d1 background monitoring enable and digital filtering for all channels:
  data_val = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 8) | (1 << 9) | (1 << 10);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_TSENSE_BG_EN, data_val, data_val); 
  if(ret != 0)
    return ret;

  //vin background int, vin0, vin1, vin2, vin3:
  data_val = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_BG_EN, data_val, data_val); 
  if(ret != 0)
    return ret;

  //vin filter int, vin0, vin1, vin2, vin3:
  data_val = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_VINX_FILTER, data_val, data_val); 
  if(ret != 0)
    return ret;

  //rs0 alarm high and low limits:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RS0_MON_HL, (uint16_t)(4096*25.0/(50.0*ADC_REF)-0.5) << 4);
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RS0_MON_LL, (uint16_t)(4096*5.0/(50.0*ADC_REF)-0.5) << 4);
  if(ret != 0)
    return ret;

  //bipolar dac range: -5..0V:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0_OFFSET, (0b10 << 4)); 
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1_OFFSET, (0b10 << 4)); 
  if(ret != 0)
    return ret;

  //ramp time:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RAMP_TIME_0, 0xffff); 
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RAMP_TIME_1, 0xffff);
  if(ret != 0)
    return ret;

  //Closed loop time constants and fast-ramp disable (better transient in capacitive load):
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_CL_FR_IT, (0b111 << 0) | (0 << 3) | (0b111 << 4) | (0 << 7));
  if(ret != 0)
    return ret;

  //
  data_val = (0 << 0) | (0 << 1) | (0 << 2) | (0 << 3);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_VINX_RANGE0, data_val, data_val);
  if(ret != 0)
    return ret;

  //
  data_val = (1 << 0) | (1 << 1) | (1 << 2) | (1 << 3);
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_VINX_RANGE1, data_val, data_val);
  if(ret != 0)
    return ret;
  delay(500);
  ////bipolar dac0, dac1 can be clamped by SLEEP0 pin
  //ad7293_spi_write(ad7293_obj, AD7293_REG_DAC_SNOOZE_O, (1 << 4) | (1 << 5)); 

  //PA on
  if(pa_on){
    psu_en(1);
    delay(1000);

    data_val = (1 << PA_ON_BIT);
    ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_PA_ON_CTRL, data_val, data_val); 
    if(ret != 0)
      return ret;

    delay(1000);    
  }

  //closed loop ch0, ch1 enable
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_INTEGR_CL, (1 << CL0_BIT) | (1 << CL1_BIT) | (1 << INT_CL_LIMIT_CH0) | (1 << INT_CL_LIMIT_CH1));
  if(ret != 0)
    return ret;

  //writing dac registers
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT0, (get_dac_value(target_amps, RSENSE0) << 4));
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_BI_VOUT1, (get_dac_value(target_amps, RSENSE1) << 4));  
  if(ret != 0)
    return ret;

  if(pa_on){
    //Bipolar dac ch0, ch1 enable (unclamping)
    data_val = (1 << BI_VOUT0_BIT)|(1 << BI_VOUT1_BIT);
    ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_DAC_EN, data_val, data_val); 
    if(ret != 0)
      return ret;

    delay(1000);
  }

  //rsx alert to ALERT0 routing:
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_RSX_MON_ALERT0, (1 << 8) | (1 << 0)); 
  if(ret != 0)
    return ret;

  //ALERT0 to GPIO3:
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_DIGITAL_INOUT_FUNC, (1 << 3), 0);
  if(ret != 0)
    return ret;

  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_DIGITAL_OUT_EN, (1 << 3));
  if(ret != 0)
    return ret;

  
  if(pa_on){
    pa_on_state = 1;
  }
  else{
    pa_on_state = 0;
  }

  ad_monitoring_halt = 0;
  return 0;
}

int ad7293_power_off(void){

  ad_monitoring_halt = 1;
  int ret = -1;
  
  //DACs in clamp
  ret = ad7293_spi_write(ad7293_obj, AD7293_REG_DAC_EN, 0); 
  if(ret != 0)
    return ret;

  delay(100);

  //PA off:
  ret = ad7293_spi_update_bits(ad7293_obj, AD7293_REG_PA_ON_CTRL, (1 << PA_ON_BIT), 0); 
  if(ret != 0)
    return ret;

  delay(100);
  psu_en(0);

  pa_on_state = 0;
  ad_monitoring_halt = 0;
  return 0;
}

void pon(uint8_t state){
  if(state > 0){
      Serial.printf("Re-configuring...\n");

      int ret = ad7293_init_and_configure(1);
      if(ret != 0){
        ad_monitoring_halt = 2;
      }
      else{
        preamp_en(1);
      }
      
      Serial.printf("ad7293_init_and_configure() result: %d\n", ret);

      fan_en(1);

  }else{

        Serial.printf("Powering off...\n");

        int ret = ad7293_power_off();
        preamp_en(0);
        fan_en(0);
        open_loop_enable(0);

        if(ret != 0){
          ad_monitoring_halt = 2;
        }

        Serial.printf("ad7293_power_off() result: %d\n", ret);

  }

}

void setup() {
  Serial.begin(9600);
  rp2040.wdt_begin(3000); // 3-second timeout (applies to both cores)
  pinMode(ETH_RESET_PIN, OUTPUT);
  digitalWrite(ETH_RESET_PIN, LOW);
  delay(100);
  digitalWrite(ETH_RESET_PIN, HIGH);
  delay(100);

  Ethernet.init(17);  // WIZnet W5100S-EVB-Pico W5500-EVB-Pico W6100-EVB-Pico

  // initialize the ethernet device
  Ethernet.begin(mac, ip, myDns, gateway, subnet);

  delay(5000);//this delay is just for serial port init

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start listening for clients
  server.begin();

  Serial.print("Server IP address:");
  Serial.println(Ethernet.localIP());
}


void loop() {
 // wait for a new client:
  EthernetClient newClient = server.accept();//server.available();

  if (newClient) {
    for (byte i=0; i < 8; i++) {
      if (!clients[i]) {
        // Once we "accept", the client is no longer tracked by EthernetServer
        // so we must store it into our list of clients
        clients[i] = newClient;
        //digitalWrite(CONNECTION_LED_PIN, HIGH);
        
        // Send welcome message to new client
        clients[i].println("X-band transmitter version 1.0");
        break;
      }
    }
  }

  // check for incoming data from all clients
  for (byte i = 0; i < 8; i++) {
    if (clients[i] && clients[i].available() > 0) {
      int count = clients[i].read((uint8_t*)rx_buff, RX_MAX_LEN - 1);
      rx_buff[count] = '\0';  // Null-terminate

      // Special case: 'mon' command (no '=')
      if (strncmp(rx_buff, "mon", 3) == 0 && (rx_buff[3] == '\0' || rx_buff[3] == '\n' || rx_buff[3] == '\r')) {
          sprintf(response_str_buff, "mon=%d,%d,%d,%0.3f,%u,%u,%u,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f,%0.3f\n", 
                  pa_on_state, psu_pg_state, open_loop_mode, rs0_volts, 
                  (unsigned int)rs0_alert_high, (unsigned int)rs0_alert_low, (unsigned int)alert0_state,
                  isense0_amps, isense1_amps, Ug0_volts, Ug0_volts_lim-Ug0_volts, Ug1_volts, 
                  Ug1_volts_lim-Ug1_volts, temperature_degC, temperature1_degC, temperature2_degC,
                  temp_degC_Fpwr, temp_degC_Rpwr, temperature3_degC, refpwr_dbm, refpwr, 
                  fwdpwr_dbm, fwdpwr, s11_param);
          clients[i].print(response_str_buff);
          continue;  // Skip rest of loop
      }

      // For other commands (with '=')
      char *equal_sign = strchr(rx_buff, '=');
      char *cmd = rx_buff;
      char *arg = NULL;

      if (equal_sign != NULL) {
          *equal_sign = '\0';  // Split command and argument
          arg = equal_sign + 1;
          // sprintf(response_str_buff, "DEBUG: Before trimming - arg: '%s'\n", arg);
          // clients[i].print(response_str_buff);
          while (*arg && isspace(*arg)) {
              arg++;
          }
          // Trim trailing whitespace
          char *arg_end = arg + strlen(arg) - 1;
          while (arg_end >= arg && isspace(*arg_end)) {
              *arg_end-- = '\0';
          }
          // sprintf(response_str_buff, "DEBUG: After trimming - arg: '%s'\n", arg);
          // clients[i].print(response_str_buff);
      }
      if (strcmp(cmd, "pon") == 0) {
        if (arg != NULL) {
          if (strcmp(arg, "0") == 0) {
              pon(0);
              sprintf(response_str_buff, "pon=0\n");
          }else if (strcmp(arg, "1") == 0) {
              pon(1);
              sprintf(response_str_buff, "pon=1\n");
          }else{
              sprintf(response_str_buff, "ERROR: Arg must be 0 or 1\n");
          }
          clients[i].print(response_str_buff);
        } else {
          sprintf(response_str_buff, "ERROR: no argument provided");
          clients[i].print(response_str_buff);
        }
      } else if (strcmp(cmd, "olm") == 0) {
        if(pa_on_state != 1){
          if (arg != NULL) {
            if (strcmp(arg, "0") == 0) {
              open_loop_enable(0);
              sprintf(response_str_buff, "olm=0\n");
            }else if (strcmp(arg, "1") == 0) {
              sprintf(response_str_buff, "Start system before entering open loop\n");
            }else{
                sprintf(response_str_buff, "ERROR: Arg must be 0 or 1\n");
            }
              clients[i].print(response_str_buff);
          }else{
            sprintf(response_str_buff, "ERROR: no argument provided");
            clients[i].print(response_str_buff);
          }
        } else {
          if (arg != NULL) {
            if (strcmp(arg, "0") == 0) {
              open_loop_enable(0);
              sprintf(response_str_buff, "olm=0\n");
            }else if (strcmp(arg, "1") == 0) {
              open_loop_enable(1);
              sprintf(response_str_buff, "olm=1\n");
            }else{
              sprintf(response_str_buff, "ERROR: Arg must be 0 or 1\n");
            }
            clients[i].print(response_str_buff);
          }else{
            sprintf(response_str_buff, "ERROR: no argument provided");
            clients[i].print(response_str_buff);
          }
        }
      } else if (strcmp(cmd, "setcurr") == 0) {
          if (arg != NULL) {
              char *endptr;
              float new_amps = strtof(arg, &endptr);

              if (arg == endptr) {
                  sprintf(response_str_buff, "ERROR: Invalid number");
              }
              else if (*endptr != '\0' && !isspace(*endptr)) {
                  sprintf(response_str_buff, "ERROR: Extra characters");
              }
              else if (new_amps < 0.0f || new_amps > 100.0f) {
                  sprintf(response_str_buff, "ERROR: Current out of range");
              }
              else {
                  target_amps = new_amps;
                  sprintf(response_str_buff, "Current set to %0.3f", target_amps);
              }
          }
          else {
              sprintf(response_str_buff, "Current value: %0.3f", target_amps);
          }
          clients[i].print(response_str_buff);
      } else {
        Serial.println("ERROR: Unknown command");
      }
    }
  }

  // stop any clients which disconnect
  for (byte i = 0; i < 8; i++) {
    if (clients[i] && !clients[i].connected()) {
      //digitalWrite(CONNECTION_LED_PIN, LOW);
      clients[i].stop();
      clients[i] = EthernetClient();  // Clear the entry
    }
  }
  rp2040.wdt_reset();
}

void setup1() {
  pinMode(BUTTON1_PIN, INPUT_PULLUP);
  pinMode(BUTTON2_PIN, INPUT_PULLUP);
  pinMode(BUTTON3_PIN, INPUT_PULLUP); 

  pinMode(LED0_PIN, OUTPUT);
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);
  pinMode(AD_ALERT0, INPUT);
  
  pinMode(PSU_EN_PIN, OUTPUT);
  pinMode(PSU_PG_PIN, INPUT); 

  pinMode(FAN_CTL_PIN, OUTPUT);
  pinMode(PREAMP_CTL_PIN, OUTPUT);

  psu_en(0);
  fan_en(0);
  preamp_en(0);

  int ret = ad7293_init_and_configure(0);
  Serial.printf("ad7293_init_and_configure() result: %d\n", ret);

  if(ret != 0){
    ad_monitoring_halt = 2;
  }
}

void loop1() {
    int ret = -1;
    static uint8_t button1_state = 0;
    static uint8_t button2_state = 0;
    static uint8_t button3_state = 0;

    uint16_t adc_raw;

    while(1){

      if((digitalRead(BUTTON1_PIN) == LOW) && (button1_state == 0)){
        pon(1);

        button1_state = 1;
      }else if(digitalRead(BUTTON1_PIN) == HIGH){
          button1_state = 0;
      }

      if((digitalRead(BUTTON2_PIN) == LOW) && (button2_state == 0)){
        pon(0);
      
        button2_state = 1;
      }else if(digitalRead(BUTTON2_PIN) == HIGH){
          button2_state = 0;
      }

      if((digitalRead(BUTTON3_PIN) == LOW) && (button3_state == 0)){
        if(open_loop_mode > 0){
          open_loop_enable(0);
        }else{
          open_loop_enable(1);
        }
          
        button3_state = 1;

      }else if(digitalRead(BUTTON3_PIN) == HIGH){
        button3_state = 0;
      }


      if(open_loop_mode){
        digitalWrite(LED1_PIN, HIGH);
      }else{
        digitalWrite(LED1_PIN, LOW);
        Ug0_volts_lim = 0;
        Ug1_volts_lim = 0;
      }

      if(ad_monitoring_halt == 0){//if in halt state, ether AD is not ready or is used in other loop
        ad7293_spi_read(ad7293_obj, AD7293_REG_RS0_MON, &adc_raw);
        rs0_raw_to_voltage(adc_raw, &rs0_volts);

        ad7293_spi_read(ad7293_obj, AD7293_REG_ISENSE_0, &adc_raw);
        isense_raw_to_current(adc_raw, &isense0_amps, RSENSE0);

        ad7293_spi_read(ad7293_obj, AD7293_REG_ISENSE_1, &adc_raw);
        isense_raw_to_current(adc_raw, &isense1_amps, RSENSE1);
        
        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT0_MON, &adc_raw);
        bi_vout_raw_to_voltage(adc_raw, &Ug0_volts);

        ad7293_spi_read(ad7293_obj, AD7293_REG_BI_VOUT1_MON, &adc_raw);
        bi_vout_raw_to_voltage(adc_raw, &Ug1_volts);

        ad7293_spi_read(ad7293_obj, AD7293_REG_RSX_MON_ALERT, &rsx_alerts);
        rs0_alert_high = (rsx_alerts>>8)&0x1;
        rs0_alert_low = (rsx_alerts>>0)&0x1;
        alert0_state = 0;

        ad7293_spi_read(ad7293_obj, AD7293_REG_TSENSE_INT, &adc_raw);
        tsense_raw_to_celsius(adc_raw, &temperature3_degC);
        ad7293_spi_read(ad7293_obj, AD7293_REG_TSENSE_D0, &adc_raw);
        tsense_raw_to_celsius(adc_raw, &temperature1_degC);
        ad7293_spi_read(ad7293_obj, AD7293_REG_TSENSE_D1, &adc_raw);
        tsense_raw_to_celsius(adc_raw, &temperature2_degC);
        
        ad7293_spi_read(ad7293_obj, AD7293_REG_VIN0, &adc_raw);
        vin_raw_to_celsius(adc_raw, &temp_degC_Rpwr);
        ad7293_spi_read(ad7293_obj, AD7293_REG_VIN1, &adc_raw);
        adc_to_dbm_ref(adc_raw, &refpwr_dbm);
        ad7293_spi_read(ad7293_obj, AD7293_REG_VIN2, &adc_raw);
        vin_raw_to_celsius(adc_raw, &temp_degC_Fpwr);
        ad7293_spi_read(ad7293_obj, AD7293_REG_VIN3, &adc_raw);
        adc_to_dbm_fwd(adc_raw, &fwdpwr_dbm);
        dbm_to_watts(refpwr_dbm, &refpwr);
        dbm_to_watts(fwdpwr_dbm, &fwdpwr);
        ref_fwd_s11(refpwr, fwdpwr, &s11_param);

        if(digitalRead(AD_ALERT0)){
          alert0_state = 1;
          pa_on_state = 0;
          digitalWrite(LED2_PIN, HIGH);
        }else{
          digitalWrite(LED2_PIN, LOW);
        }     
      }else if(ad_monitoring_halt == 2){
        Serial.printf("ad7293 init failed ...\n");
      }
      else{
        Serial.printf("Monitoring halted...\n");
      }

      psu_pg_state = (int)digitalRead(PSU_PG_PIN);

      float voltage = analogRead(TEMP_SENSOR_PIN)*(3300 / 1023.0);
      temperature_degC = voltage / 10;

      digitalWrite(LED0_PIN, LOW);
      delay(500);
      digitalWrite(LED0_PIN, HIGH);
      delay(500);     
  }
}
