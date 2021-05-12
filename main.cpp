#include "mbed.h"
#include "mbed_rpc.h"
#include "stm32l475e_iot01_accelero.h"
#include "uLCD_4DGL.h"
#include "mbed_events.h"
#include "config.h"
#include <iostream>

#include "magic_wand_model_data.h"
#include "accelerometer_handler.h"
#include "config.h"

#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"

#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"

#include <cmath>

using namespace std::chrono;
using namespace std;


double angle_det;

InterruptIn button(USER_BUTTON);

DigitalOut led3(LED3);
uLCD_4DGL uLCD(D1, D0, D2);
int led = 0;
int angle_list[5] = {30, 35, 40, 45, 50};
int mode = 0;
int angle_index;
int public_ctrl = 0;
int keep_in_loop = 1;

int time_length = 20;

int16_t DataXYZ[10][3] = {0};

Thread t;
EventQueue queue(32 * EVENTS_EVENT_SIZE);
Thread k;
EventQueue queue2(32 * EVENTS_EVENT_SIZE);
Thread w;
EventQueue queue3(32 * EVENTS_EVENT_SIZE);
/************************
 * MQTT
 * **********************/
// GLOBAL VARIABLES
WiFiInterface *wifi;
//InterruptIn btn2(USER_BUTTON);
//InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

char* topic1 = "Mbed1";
char* topic2 = "Mbed2";
char* topic;

Thread mqtt_thread(osPriorityHigh);
EventQueue mqtt_queue;

void messageArrived(MQTT::MessageData& md);
void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client);
void close_mqtt();
void connectWIFI();



/************************
 * tensorflow
 * **********************/
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];


void flip(Arguments *in, Reply *out) {mode = !mode;}
void flip_2() {mode = !mode;}

BufferedSerial pc(USBTX, USBRX);

void start_tiltDetective_threat(Arguments *in, Reply *out);
void tiltDetective();

void start_guesture_threat(Arguments *in, Reply *out);
void guesture_IU();//Arguments *in, Reply *out);

RPCFunction RPCguesture_IU(&start_guesture_threat, "guesture_IU");
RPCFunction changeMode(&flip, "broker");
RPCFunction tiltMode(&start_tiltDetective_threat, "tilt");

void uLCD_print(int angle);

/********************************************************************************/
/********************************************************************************/
/********************************************************************************/

int main() {

	BSP_ACCELERO_Init();
	t.start(callback(&queue, &EventQueue::dispatch_forever));
	k.start(callback(&queue2, &EventQueue::dispatch_forever));
	//w.start(callback(&queue3, &EventQueue::dispatch_forever));

	queue3.call(&connectWIFI);




	char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");


  	wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return -1;
    }


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return -1;
    }


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.254";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
     data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic1, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }
    if (client.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

    cout << "wating\n";
	  mqtt_thread.start(callback(&mqtt_queue, &EventQueue::dispatch_forever));
    //button.rise(&flip_2);//mqtt_queue.call(publish_message, &client));
    button.rise(mqtt_queue.event(&publish_message, &client));

	while(1) {
        memset(buf, 0, 256);
        for (int i = 0; ; i++) {
            char recv = fgetc(devin);
            if (recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        printf("%s\r\n", buf);
        RPC::call(buf, outbuf);

      //if (public_ctrl) {
      // mqtt_queue.call(&publish_message, &client);
       // public_ctrl = 0;
      //}
    }

}






















/**************************************************
 * 
 *  
 *              function defination
 * 
 * 
 * 
 * 
 * 
 * ************************************************/



int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}


void guesture_IU() {//Arguments *in, Reply *out) {

  // Whether we should clear the buffer next time we fetch data
  bool should_clear_buffer = false;
  bool got_data = false;

  // The gesture index of the prediction
  int gesture_index;


    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client2(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.254";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return ;//-1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed2\0";

    if ((rc = client2.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client2.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }




  // Set up logging.
  static tflite::MicroErrorReporter micro_error_reporter;
  tflite::ErrorReporter* error_reporter = &micro_error_reporter;





  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    error_reporter->Report(
        "Model provided is schema version %d not equal "
        "to supported version %d.",
        model->version(), TFLITE_SCHEMA_VERSION);
    return ;//-1;
  }

  // Pull in only the operation implementations we need.
  // This relies on a complete list of all the ops needed by this graph.
  // An easier approach is to just use the AllOpsResolver, but this will
  // incur some penalty in code space for op implementations that are not
  // needed by this graph.
  static tflite::MicroOpResolver<6> micro_op_resolver;
  micro_op_resolver.AddBuiltin(
      tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
      tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
  micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

  // Build an interpreter to run the model with
  static tflite::MicroInterpreter static_interpreter(
      model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
  tflite::MicroInterpreter* interpreter = &static_interpreter;

  // Allocate memory from the tensor_arena for the model's tensors
  interpreter->AllocateTensors();

  // Obtain pointer to the model's input tensor
  TfLiteTensor* model_input = interpreter->input(0);
  if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
      (model_input->dims->data[1] != config.seq_length) ||
      (model_input->dims->data[2] != kChannelNumber) ||
      (model_input->type != kTfLiteFloat32)) {
    error_reporter->Report("Bad input tensor parameters in model");
    return ;//-1;
  }

  int input_length = model_input->bytes / sizeof(float);

  TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
  if (setup_status != kTfLiteOk) {
    error_reporter->Report("Set up failed\n");
    return ;//-1;
  }

  error_reporter->Report("Set up successful...\n");

  BSP_ACCELERO_Init();
  while (mode == 0) {

    // Attempt to read new data from the accelerometer
    got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);
    // If there was no ew data,
    // don't try to clear the buffer again and wait until next time
    if (!got_data) {
      should_clear_buffer = false;
      continue;
    }



    // Run inference, and report any error

    TfLiteStatus invoke_status = interpreter->Invoke();
    if (invoke_status != kTfLiteOk) {
      error_reporter->Report("Invoke failed on index: %d\n", begin_index);
      continue;
    }

    // Analyze the results to obtain a prediction
    gesture_index = PredictGesture(interpreter->output(0)->data.f);

    // Clear the buffer next time we read data
    should_clear_buffer = gesture_index < label_num;

    // us gesture index to control the index


    if(gesture_index == 0) {
      int k = 0;
      while(k++<10) {
        BSP_ACCELERO_AccGetXYZ(DataXYZ[k]);
        ThisThread::sleep_for(10ms);
      }
      cout << DataXYZ[0][0] << "," << DataXYZ[0][1] << "," << DataXYZ[0][2] << endl;
      
    } else if (gesture_index == 1) {
            while(k++<10) {
        BSP_ACCELERO_AccGetXYZ(DataXYZ[k]);
        ThisThread::sleep_for(10ms);
      }
      cout << DataXYZ[0][0] << "," << DataXYZ[0][1] << "," << DataXYZ[0][2] <<endl;
      
    } else if (gesture_index == 2) {
            while(k++<10) {
        BSP_ACCELERO_AccGetXYZ(DataXYZ[k]);
        ThisThread::sleep_for(10ms);
      }
      cout << DataXYZ[0][0] << "," << DataXYZ[0][1] << "," << DataXYZ[0][2] <<endl;
      
    }



    // Produce an output
    if (gesture_index < label_num) {
      error_reporter->Report(config.output_message[gesture_index]);
    }
  }
  cout << " confirm index :" << angle_list[angle_index % 5] << endl;


}

void uLCD_print(int angle)
{
    uLCD.background_color(WHITE);
    uLCD.textbackground_color(WHITE);
    uLCD.color(BLUE);
    uLCD.cls();

    uLCD.text_width(4); //4X size text
    uLCD.text_height(4);
    uLCD.color(GREEN);
    uLCD.locate(1,2);
    //uLCD.printf("%2d",i);
    uLCD.cls();
    uLCD.text_width(4); //4X size text
    uLCD.text_height(4);
    
    uLCD.locate(1,2);
    uLCD.printf("%2d",angle);
    
    /*
    for ( i; i>=0; i--) {

        ThisThread::sleep_for(500ms);
    }*/
}

/**************************************************
 * 
 *  MQTT
 * 
 * ************************************************/

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_message(MQTT::Client<MQTTNetwork, Countdown>* client) {

    message_num++;
    MQTT::Message message;
    char buff[100];

    if(mode == 0) {
      topic = topic1;
      printf("topic1\n");
      sprintf(buff, "%d\r\n", angle_list[angle_index]);
    } else {
      topic = topic2;
      printf("topic2\n");
      sprintf(buff, "%d\r\n",int(angle_det));
    }

    
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buff;
    message.payloadlen = strlen(buff) + 1;
    


    int rc = client->publish(topic, message);

    //public_ctrl = 0;
    ThisThread::sleep_for(100ms);


}

void close_mqtt() {
    closed = true;
}

void connectWIFI() {


	while(1) {ThisThread::sleep_for(500ms);}
}


void start_guesture_threat (Arguments *in, Reply *out) {
    queue.call(&guesture_IU);
	//.start(callback(&guesture_IU, Arguments *in, Reply *out, &EventQueue::dispatch_forever));
}

void start_tiltDetective_threat (Arguments *in, Reply *out) {
    queue2.call(&tiltDetective);
}
/*
void tiltDetective() {
  int16_t pDataXYZ[3] = {0};
  BSP_ACCELERO_AccGetXYZ(pDataXYZ);
  char buff[100] = {0};
  sprintf(buff, "X, Y, Z: %d, %d, %d\r\n", pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);

  while(mode) {
    
  }


}
*/
void tiltDetective() {
    //for Accelerometer
    int16_t pDataXYZ_init[3] = {0};
    int16_t pDataXYZ[3] = {0};
    double mag_A;
    double mag_B;
    double cos;
    double rad_det;
    //double angle_det;
    //int idR[32] = {0};
    //int indexR = 0;
    NetworkInterface* net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client2(mqttNetwork);

    //TODO: revise host to your IP
    const char* host = "192.168.43.254";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"),  (sockAddr.get_port() ? sockAddr.get_port() : 0) ); //check setting

    int rc = mqttNetwork.connect(sockAddr);//(host, 1883);
    if (rc != 0) {
            printf("Connection error.");
            return ;//-1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed2\0";

    if ((rc = client2.connect(data)) != 0){
            printf("Fail to connect MQTT\r\n");
    }
    if (client2.subscribe(topic2, MQTT::QOS0, messageArrived) != 0){
            printf("Fail to subscribe\r\n");
    }

/*
    wifi = WiFiInterface::get_default_instance();
    if (!wifi) {
            printf("ERROR: No WiFiInterface found.\r\n");
            return ;//-1;
    };


    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0) {
            printf("\nConnection error: %d\r\n", ret);
            return ;//-1;
    }*/







    printf("angle detection mode\r\n");
    printf("Place the mbed on table\r\n");
    ThisThread::sleep_for(2000ms);
    for (int i=0; i<5; i++) {
        led3 = 1;                            // use LED3 to show the initialization process
        ThisThread::sleep_for(100ms);
        led3 = 0;
        ThisThread::sleep_for(100ms);
    }
    
    BSP_ACCELERO_AccGetXYZ(pDataXYZ_init);
    printf("reference acceleration vector: (%d, %d, %d)\r\n", pDataXYZ_init[0], pDataXYZ_init[1], pDataXYZ_init[2]);

    printf("Tilt the mbed after LEDs\r\n");
    ThisThread::sleep_for(2000ms);
    for (int i=0; i<5; i++) {
        led3 = 1;                            // use LED3 to show the initialization process
        ThisThread::sleep_for(100ms);
        led3 = 0;
        ThisThread::sleep_for(100ms);
    }
    
                // tile Angle_Detection mode
    while (mode) {
        BSP_ACCELERO_AccGetXYZ(pDataXYZ);
        printf ("Angle Detection: %d %d %d\r\n",pDataXYZ[0], pDataXYZ[1], pDataXYZ[2]);
        mag_A = sqrt(pDataXYZ_init[0] * pDataXYZ_init[0] + pDataXYZ_init[1] * pDataXYZ_init[1] + pDataXYZ_init[2] * pDataXYZ_init[2]);
        mag_B = sqrt(pDataXYZ[0] * pDataXYZ[0] + pDataXYZ[1] * pDataXYZ[1] + pDataXYZ[2] * pDataXYZ[2]);
        cos = ((pDataXYZ_init[0] * pDataXYZ[0] + pDataXYZ_init[1] * pDataXYZ[1] + pDataXYZ_init[2] * pDataXYZ[2])/(mag_A)/(mag_B));
        rad_det = acos(cos);
        angle_det = 180.0 * rad_det/3.1415926;
        printf("angle_det = %f\r\n", angle_det);
        uLCD_print(angle_det);
        // queue.call(another print function for uLCD)

        if (angle_det > angle_list[angle_index]) {
            //public_ctrl = 1;
            mqtt_queue.call(&publish_message, &client2);
            printf("over tilting\n");
        }
        ThisThread::sleep_for(1000ms);
    }






}



