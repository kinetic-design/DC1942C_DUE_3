//
// DC1942C_DUE_3.cpp
//
// Written by Steve Schulz on 25 September 2018
// Kinetic Design LLC, Huntington Beach California
//
// Sketch for an SAM8X DUE GEVCU board that controls the undervoltage
// flag (UVF) and overvoltage flag (OVF)  lines of a BRUSA NLG513 charger
// PFC charger based on voltage data received from a stack of '6804-2
// chips via the LTC6804_2_Stack class. Note that a SPDT relay is placed
// between the PFC charger's UVF and OVF lines and the Linduino
//


/*! @file
    @ingroup LTC68042
*/
#include <stdint.h>
#include <Scheduler.h>
#include <Linduino.h>
#include <DC1942C_Stack.h>
#include "UserInterface.h"

#define MAX_FLOAT 3.4028235E+38
#define FLOAT_PRINT_PRECISION 3

// Digital pins that control the state of the undervoltage and
// overvoltage lines on the PFC charger
#define UV_FLAG_PIN 3
#define OV_FLAG_PIN 2

// Cell voltage limits
// (Samsung 18650 cell, 3.6V nominal)
#define UNDERVOLTAGE_THRESHOLD 3.0  // Volts
#define OVERVOLTAGE_THRESHOLD  4.0  // Volts
// The maximum difference between two cell voltages
// for which the voltages are considered equal
#define VOLTAGE_EQUAL_MARGIN  0.001 // Volts 0.05 nominal
// For hysteresis; should not be significantly greater than VOLTAGE_EQUAL_MARGIN
#define VOLTAGE_THRESHOLD_EPSILON 0.002// Volts

// Macros to set and clear overvoltage and undervoltage
// lines on the PFC charger; depend on relay wiring
#define SET_OVERVOLTAGE() digitalWrite( OV_FLAG_PIN, HIGH );  Serial.println( "SET OV" )
#define CLEAR_OVERVOLTAGE() digitalWrite( OV_FLAG_PIN, LOW ); Serial.println( "CLEAR OV" )
#define SET_UNDERVOLTAGE() digitalWrite( UV_FLAG_PIN, HIGH ); Serial.println( "SET UV" )
#define CLEAR_UNDERVOLTAGE() digitalWrite( OV_FLAG_PIN, LOW );Serial.println( "CLEAR UV" )
// Macros that determine the states of the OVF and UVF lines
#define OVERVOLTAGE_ASSERTED() digitalRead( OV_FLAG_PIN ) == HIGH
#define UNDERVOLTAGE_ASSERTED() digitalRead( UV_FLAG_PIN ) == HIGH

// Rate of PC-Linduino serial communication
#define SERIAL_BAUD 9600//115200
// Number of '6804 ICs on the stack connected to the Linduino
#define NUM_ICS 4//6
// Number of analog-input pins utilized as thermistors on the DC1942C boards
#define NUM_THERMISTORS 8
// Adresses of the '6804 ICs
const uint8_t IC_ADDRESSES[NUM_ICS] = { 0, 1, 2, 3 };// 3, 4, 6, 7 };
// Analog input pins that accomidate thermistors on the DC1942C boards
const uint8_t THERMISTOR_PINS[NUM_THERMISTORS] = {
  DC1942C_AI0, DC1942C_AI1, DC1942C_AI2, DC1942C_AI3, DC1942C_AI4, DC1942C_AI5, DC1942C_AI6, DC1942C_AI7
};
// Thermistor constants
#define THERMISTOR_B_PARAMETER 3950
#define THERMISTOR_AMBIENT_TEMPERATURE_KELVIN 297.6
const float THERMISTOR_AMBIENT_RESISTANCES_OHMS[NUM_ICS][NUM_THERMISTORS] PROGMEM = {
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730}};
/*  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730},
  {10730, 10730, 10730, 10730, 10730, 10730, 10730, 10730}
};*/

// Array to hold chip configuration words; make certian
// to initialize to zero. Declared as a global variable so
// that IC configuration can be verified periodically in loop(),
// but initialized in setup()
uint16_t configData[LTC6804_NUM_CONFIG_WORDS] = { 0 };

// Construct LTC6804_2_Stack object; the SPI chip select (CS)
// pin assocaited with the stack by is the only argument
DC1942C_Stack stack( SPI_CS );

// Arrays to hold data read from ICs
// Defined as globals so that the Arduino IDE shows
// memory usage
float cellVoltages[NUM_ICS][LTC6804_NUM_CELLS];
float minCellVoltage, maxCellVoltage, deltaCellVoltage, totalCellVoltage;
float temperatures_celsius[NUM_ICS][NUM_THERMISTORS];
float socSUM;

bool DEBUG = 0;

// Variables to hold data used in verifying IC configuration, voltage, ad GPIO data
bool cvOk, analogOk, configOk;
bool configStatus[NUM_ICS];

// Cell balancing
bool balanceCells = false;
bool stopBalancing;

int rdError = 0;
int ledpwr  = 1;
int LED13   = 13;  // diagnostic LED heartbeat

uint16_t user_command;

// Function Prototype Declarations
void print_voltage();
void loop();
void task1();
void task2();
void task3();

void print_reg_group( const uint16_t* const regDataPtr, const uint8_t grpSize, const uint8_t numGrps );
void print_float_matrix( float* arr, uint8_t nRows, uint8_t nCols, uint8_t prec, char* unit );
void print_bool_array( bool* arr, uint8_t len, char* trueVal, char* falseVal );
void StackConfig();
void NOTBALANCING();
void BALANCING();
void CVOK();
void AOK();
void CONFIGSTATUS();
void CVOKData();
void run_command(uint16_t cmd);
float float_matrix_min( float matrix[][LTC6804_NUM_CELLS], const size_t nrow );
float float_matrix_max( float matrix[][LTC6804_NUM_CELLS], const size_t nrow );
float float_matrix_sum( float matrix[][LTC6804_NUM_CELLS], const size_t nrow );

/******************************************************************************/
/************************************* SETUP **********************************/
void setup()
{
    Serial.begin( SERIAL_BAUD );  // Initialize PC-GEVCU serial communications
    while ( !Serial.available() );


    pinMode( LED13, OUTPUT );
    pinMode( SPI_CS, OUTPUT );
    pinMode( UV_FLAG_PIN, OUTPUT ); // Undervoltage pin
    pinMode( OV_FLAG_PIN, OUTPUT ); // Overvoltage pin
    CLEAR_UNDERVOLTAGE();
    CLEAR_OVERVOLTAGE();

    // Setup stack-specific hardware; this initializes the stack's
    // chip-select line
    stack.setup();
    // Call after all stack-object setup() functions have been called;
    // this initializes SPI and timer hardware shared between stacks
    DC1942C_Stack::start();

    // Load array with configuration words generated from the supplied
    // parameters
    stack.prepare_config(
      configData,
      PD_NONE,              // GPIO pulldown enable
      REF_ON,               // REFON bit
      ADC_OPT_1,            // ADCOPT bit
      // Use macros to generate codes corresponding to under- and over-voltage values
      UV_THRESHOLD_CODE(UNDERVOLTAGE_THRESHOLD),// Undervoltage threshold
      OV_THRESHOLD_CODE(OVERVOLTAGE_THRESHOLD), // Overvoltage threshold
      D_NONE,               // Cell discharge shunt enable
      DISCHARGE_TIMEOUT_OFF // Cell discharge timer
    );
    Serial.println( "PREPARED CONFIGURATION" );
    print_reg_group( configData, LTC6804_NUM_CONFIG_WORDS_PER_REG, 1 );
    Serial.println();

    // Attempt to write configuration data to all '6804 ICs on the stack
    if( !stack.config_all( NUM_ICS, IC_ADDRESSES, configData ) )
    {
      // Could not verify all ICs configured properly; halt SPI communications
      // and output a message to the user
      Serial.println( "Failed to verify IC configuration! Verify connections and software constants (e.g. IC count and addresses)." );
      DC1942C_Stack::stop();
    } // end if

    StackConfig();

  //  Scheduler.startLoop(task1);
    Scheduler.startLoop(loop);
    Scheduler.startLoop(task1);
    Scheduler.startLoop(task2);
}


/************** LED blinker task *********************************************/
void loop()
{
    if (ledpwr){
        digitalWrite(LED13, HIGH);
        delay(250); // delay insures other tasks run
        digitalWrite(LED13, LOW);
        delay(250);
    }
    else {
        digitalWrite(LED13, LOW);
        delay(100);
    }
    if (balanceCells) {
        BALANCING();
    }
    else if (!balanceCells){
        NOTBALANCING();
    }
}


/************** Command Line Control task ************************************/
void task1()
{
  if (Serial.available())
    {
        user_command = read_int();    // Read the user command
        Serial.println(user_command);
        run_command(user_command);
    }
    delay(100);
    yield();
}


/************** Stack Data Report task ***************************************/
void task2()
{
//    while ( true ) {                // continuous
//    while (!Serial.available()) {}  // using <return>
    if (Serial.available()) {

        StackConfig();

      // Output collected data

      if( cvOk )
      {
          CVOK();
      } // end if
      else
      {
        Serial.println( "Could not fetch cell-voltage data." );
      } // end else

      if( analogOk )
      {
        AOK;
      } // end if
      else
      {
        Serial.println( "Could not fetch analog input data." );
      } // end else

      CONFIGSTATUS();
    } // end loop
    delay(500);
//    yield();
} // add for continuous


/************** Menu Function Task *******************************************/
void run_command(uint16_t cmd)
{
  //int8_t error = 0;

  char input = 0;
  switch (cmd)
  {
    case 0:
      print_menu();
      break;

    case 1:
      //Serial.println("case 1 Balancing Off");
      balanceCells = false;
      NOTBALANCING();
      break;

    case 2:
      //Serial.println("case 2 Balancing On");
      balanceCells = true;
      BALANCING();
      break;

    case 3:
      //Serial.println("case 3");
      CVOK();
      break;

    case 4:
      //Serial.println("case 4");
      AOK();
      break;

    case 5:
      //Serial.println("case 5");
      CONFIGSTATUS();
      break;

    case 6:
      Serial.println("+-----------------------------------------------+");
      Serial.println(" Samsung 18650 Cell Voltage Limits, 3.6v nominal");
      Serial.println("+-----------------------------------------------+");
      Serial.print("UnderVoltage Threshold   : ");
      Serial.print(UNDERVOLTAGE_THRESHOLD,3);Serial.println(" Volts");

      Serial.print("OverVoltage Threshold    : ");
      Serial.print( OVERVOLTAGE_THRESHOLD, 3 ); Serial.println(" Volts");

      // The maximum difference between two cell voltages
      // for which the voltages are considered equal
      Serial.print("Voltage Equal Margin     : ");
      Serial.print( VOLTAGE_EQUAL_MARGIN, 3 ); Serial.println(" Volts");

      // For hysteresis; should not be significantly greater than VOLTAGE_EQUAL_MARGIN
      Serial.print("Voltage Threshold Epsilon: ");
      Serial.print( VOLTAGE_THRESHOLD_EPSILON, 3 ); Serial.println(" Volts");
      break;

    case 7:
      Serial.println("transmit 'm' to quit");
      //wakeup_sleep();
      //LTC6804_wrcfg(TOTAL_IC,tx_cfg);
      while (input != 'm')
      {
          CVOKData();
          delay(1000);
      }
      //print_menu();
      break;

    default:
      Serial.println("Incorrect Option");
      break;
  }
}

void StackConfig() {
    // Broadcast ADC start commands to all ICs on the stack
    // NOTE: ADC start commands cannot be issued one-after-the-other; must wait
    // for the previous conversion to finish before another can be started
    stack.start_cell_voltage_adc( BROADCAST, ADC_MODE_NORMAL, ADC_PERMIT_DISCHARGE_FALSE, ADC_CELL_CHANNEL_ALL );
    stack.adc_delay( CELL_VOLTAGE, ADC_OPT_1, ADC_MODE_NORMAL, ADC_CELL_CHANNEL_ALL );

    // Fetch cell-voltage and auxillary data from ICs
    cvOk = stack.get_all_cell_voltages( NUM_ICS, IC_ADDRESSES, cellVoltages );
    minCellVoltage = float_matrix_min( cellVoltages, NUM_ICS );
    maxCellVoltage = float_matrix_max( cellVoltages, NUM_ICS );
    deltaCellVoltage = maxCellVoltage - minCellVoltage;
    socSUM = float_matrix_sum( cellVoltages, NUM_ICS );

    //analogOk = 0;  // No analog
    analogOk = stack.thermistor_read_all( NUM_ICS, IC_ADDRESSES, NUM_THERMISTORS, THERMISTOR_PINS,
        ADC_OPT_1, ADC_MODE_NORMAL,THERMISTOR_B_PARAMETER, THERMISTOR_AMBIENT_TEMPERATURE_KELVIN,
        THERMISTOR_AMBIENT_RESISTANCES_OHMS[0],temperatures_celsius[0] );

    // Verify ICs have retained proper configuration
    configOk = stack.verify_all_config( NUM_ICS, IC_ADDRESSES, configData, configStatus );
}

// Voltage Monitoring and Cell Balancing Routines
void NOTBALANCING()
{
        if (DEBUG)
            Serial.println( "NOT BALANCING balancing turned off" );

        for( size_t icIndex = 0; icIndex < NUM_ICS; icIndex++ )
        {
            for( size_t cellIndex = 0; cellIndex < LTC6804_NUM_CELLS; cellIndex++ )
            {
                if( cellVoltages[icIndex][cellIndex] > OVERVOLTAGE_THRESHOLD )
                {
                    // A cell voltage has exceeded the overvoltage threshold;
                    // set overvoltage flag on charger and start cell balancing
                    Serial.println( " SET OVERVOLTAGE ");
                    SET_OVERVOLTAGE();
                    balanceCells = true;
                } // end if
            } // end for
        } // end for
  //} // end if;

}


void BALANCING()
{

    stopBalancing = true;

    if (DEBUG)
        Serial.println( "BALANCING" );

    for( size_t icIndex = 0; icIndex < NUM_ICS; icIndex++ )
    {
        uint16_t configData[LTC6804_NUM_CV_REGS];
        uint16_t dischargeMap = 0;

        if( !stack.get_raw_config( IC_ADDRESSES[icIndex], configData ) )
        {
            Serial.print( "WARN! Failed reading discharge map for IC with the address '" );
            Serial.print( IC_ADDRESSES[icIndex] );
            Serial.println( "'." );
            continue;
        } // end if

        dischargeMap = stack.strip_data( configData, CONFIG_CELL_DISCHARGE_CTL );

        for( size_t cellIndex = 0; cellIndex < LTC6804_NUM_CELLS; cellIndex++ )
        {
            if( cellVoltages[icIndex][cellIndex] > minCellVoltage + VOLTAGE_EQUAL_MARGIN )
            {
                // Set discharge
                dischargeMap |= (uint16_t)(1 << cellIndex);
                Serial.print("Cell[");
                Serial.print(cellIndex);  // print the overvoltage balancing cell
                Serial.print("], ");
            } // end if
            else if( cellVoltages[icIndex][cellIndex] <= minCellVoltage + VOLTAGE_EQUAL_MARGIN - VOLTAGE_THRESHOLD_EPSILON )
            {
                // Clear discharge
                dischargeMap &= ~((uint16_t)(1 << cellIndex));
            } // end if
              // (don't modify the discharge state of cells with voltages inside the hysteresis range)
        } // end for

            if( dischargeMap )
            {
                stopBalancing = false;
            } // end if

            stack.bind_data( configData, CONFIG_CELL_DISCHARGE_CTL, dischargeMap );

            if( !stack.config( IC_ADDRESSES[icIndex], configData ) )
            {
                Serial.print( "WARN! Failed to write discharge map for the IC with the address '" );
                Serial.print( IC_ADDRESSES[icIndex] );
                Serial.println( "'." );
                continue;
             } // end if
        } // end for
}

void CVOK()
{
    Serial.println( "\nCELL VOLTAGES:" );
    Serial.println( "Rows <=> ICs / Columns <=> Cells" );
    print_float_matrix( cellVoltages[0], NUM_ICS, LTC6804_NUM_CELLS, FLOAT_PRINT_PRECISION, "V" );
    Serial.print( "\nSOC: " );
    Serial.print( socSUM , 3 );
    Serial.print( "\nMIN VOLTAGE: " );
    Serial.println( minCellVoltage, FLOAT_PRINT_PRECISION );
    Serial.print( "MAX VOLTAGE: " );
    Serial.println( maxCellVoltage, FLOAT_PRINT_PRECISION );
    Serial.print( "VOLTAGE DIFF: ");
    Serial.println( deltaCellVoltage, FLOAT_PRINT_PRECISION);
}

void CVOKData()
{
    print_float_matrix( cellVoltages[0], NUM_ICS, LTC6804_NUM_CELLS, FLOAT_PRINT_PRECISION, NULL );
}

void AOK() {
    Serial.println( "TEMPERATURES" );
    Serial.println( "Rows <=> ICs / Columns <=> Thermistors" );
    print_float_matrix( temperatures_celsius[0], NUM_ICS, NUM_THERMISTORS, FLOAT_PRINT_PRECISION, "*C" );
}

void CONFIGSTATUS()
{
    Serial.println();
    Serial.print( "CHIP CONFIGURATION STATUS: " );
    Serial.println( configOk ? "No errors detected." : "Error(s) detected!" );
    Serial.println( "Columns <=> ICs" );
    print_bool_array( configStatus, NUM_ICS, "OK", "ERR" );
    Serial.println();
}

bool any( uint16_t* arr, size_t len )
{
  for( ; len > 0; len-- )
  {
    if( arr[len - 1] )
    {
      return true;
    } // end if
  } // end for

  return false;
} // end any

float float_matrix_min( float matrix[][LTC6804_NUM_CELLS], const size_t nrow )
{
  float smallest = MAX_FLOAT;

  for( size_t row = 0; row < nrow; row++ )
  {
    for( size_t col = 0; col < LTC6804_NUM_CELLS; col++ )
    {
      float val = matrix[row][col];

      if( val < smallest )
      {
        smallest = val;
      } // end if
    } // end for
  } // end for

  return smallest;
} // end float_matrix_min

float float_matrix_max( float matrix[][LTC6804_NUM_CELLS], const size_t nrow )
{
  float largest = 0.0;

  for( size_t row = 0; row < nrow; row++ )
  {
    for( size_t col = 0; col < LTC6804_NUM_CELLS; col++ )
    {
      float val = matrix[row][col];

      if( val > largest )
      {
        largest = val;
      } // end if
    } // end for
  } // end for

  return largest;
} // end float_matrix_max

float float_matrix_sum( float matrix[][LTC6804_NUM_CELLS], const size_t nrow )
{
    float sum = 0.0;

    for ( size_t row = 0; row < nrow; row++ )
    {
        for( size_t col = 0; col < LTC6804_NUM_CELLS; col++ )
        {
          float val = matrix[row][col];

          sum = sum + val;
        } // end for
    } // end for

    return sum;
}

void print_float_array( float* arr, uint8_t len, uint8_t prec, char* unit )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i], prec );
    Serial.print( unit );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_float_array

void print_bool_array( bool* arr, uint8_t len, char* trueVal, char* falseVal )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i] ? trueVal : falseVal );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_float_array

void print_binary16_array( uint16_t* arr, uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i], BIN );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_binary_array

void print_binary8_array( uint8_t* arr, uint8_t len )
{
  for( uint8_t i = 0; i < len; i++ )
  {
    Serial.print( arr[i], BIN );
    Serial.print( " " );
  } // end for

  Serial.println();
} // end print_binary_array

void print_float_matrix( float* arr, uint8_t nRows, uint8_t nCols, uint8_t prec, char* unit )
{
  for( uint8_t row = 0; row < nRows; row++ )
  {
    for( uint8_t col = 0; col < nCols; col++ )
    {
      Serial.print( arr[row * nCols + col], prec );
      Serial.print( unit );
      Serial.print( " " );
    } // end for

    Serial.println();
  } // end for
} // end print_float_matrix

void print_reg_group( const uint16_t* const regDataPtr, const uint8_t grpSize, const uint8_t numGrps )
{
  for( uint8_t grp = 0; grp < numGrps; grp++ )
  {
    for( uint8_t grpElement = grpSize * grp; grpElement < grpSize * (grp + 1); grpElement++ )
    {
      Serial.print( "0x" );
      Serial.print( regDataPtr[grpElement], HEX );
      Serial.print( " " );
    } // end for

    Serial.println();
  } // end for
} // end print_reg_group
