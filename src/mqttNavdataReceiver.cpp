/*******************************************************************************
   Edited by: 2016 Harshit Sureka <harshit.sureka@gmail.com>
 *
 * Navdata Receiver for data sent by ardrone_autonomy through the mqtt_bridge
 *
 *******************************************************************************/

#include "ros/ros.h"
#include "ros/serialization.h"
#include <mosquittopp.h>
#include <ardrone_autonomy/Navdata.h>

std::string CLIENTID("mqttNavdataReceiver");

/*
 * Extend class mosquittopp from the /usr/include/mosquittopp.h file 
 * This class provides all the basic functions for mqtt and 
 * virtual callback functions that are implemented
 */
class mqtt_bridge : public mosquittopp::mosquittopp
{
  private:
    //ros nodehandle to handle ROS Topics publishes and subscribes
    ros::NodeHandle nh_;

    //The publisher for navdata on ROS topic
    ros::Publisher navdataPub_;

  public:
    //The constructor
    mqtt_bridge(const char *id, const char *host, int port, ros::NodeHandle nh);

    //The Destructor
    ~mqtt_bridge();

    //Callback for when the mqtt client is connected
    void on_connect(int rc);

    //Callback for when the mqtt client receives a message on a subscribed topic
    void on_message(const struct mosquitto_message *message);

    //Callback for when the mqtt message succeeds in subscribing to a topic
    void on_subscribe(int mid, int qos_count, const int *granted_qos);

    //Set the image and navdata publishers over ROS. Called in the constructor.
    void initPublishers();

    //Callback redirects here when a Navdata message is received over MQTT. This function packages the data received
    //over MQTT into a navMsg format for ROS. It then publishes that message out.
    void handleNavdata(const struct mosquitto_message *message);

};


/*
 *Initialize the publishers that send the message over ROS topics. This is called in the constructor.
 */
void mqtt_bridge::initPublishers()
{
  //The publisher for ROS navdata on tum_ardrone/navdata
  navdataPub_ = nh_.advertise<ardrone_autonomy::Navdata>("tum_ardrone/navdata", 1);
}


//The Constructor
//Intializes Variables, Intializes publishers, Connects the mqtt client.
mqtt_bridge::mqtt_bridge(const char *id, const char *host, int port, ros::NodeHandle nh) : 
  mosquittopp(id),
  nh_(nh) 
{
  int keepalive = 60;
  
  //initialize the navdata and img ros publishers
  initPublishers();

  //Connect this class instance to the mqtt host and port.
  connect(host, port, keepalive);
};

//Destructor
mqtt_bridge::~mqtt_bridge()
{
}

//Callback when the mqtt client successfully connects. rc = 0 means successful connection.
void mqtt_bridge::on_connect(int rc)
{
  ROS_INFO("Connected with code %d.\n", rc);
}

void mqtt_bridge::handleNavdata(const struct mosquitto_message *message)
{

  ardrone_autonomy::Navdata navMsg;

  uint32_t file_size = message->payloadlen;
  boost::shared_array<uint8_t> ibuffer(new uint8_t[file_size]);
  
  ros::serialization::IStream istream(message->payload, file_size);
  ros::serialization::deserialize(istream, navMsg);

  navdataPub_.publish(navMsg);

  return;
}

//When we receive a mqtt message, this callback is called. It just calls the responsible function
//depending on the topic of the mqtt message that was received.
void mqtt_bridge::on_message(const struct mosquitto_message *message)
{
	if(!strcmp(message->topic, "/ardrone/navdata"))
	{
		handleNavdata(message);
	}
}

//Callback when the mosquitto library successfully subscribes to a topic
void mqtt_bridge::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
  ROS_INFO("Subscription succeeded.\n");
}

int main(int argc, char **argv)
{
  //Start with a new random client ID each time, so that prev messages aren't a hassle.
  srand(time(NULL));
  CLIENTID += std::to_string(rand());

  //Mandatory ROS INIT call for this file to be registered as a ROS NODE. 
  ros::init(argc, argv, "mqttNavdataReceiver");
  ros::NodeHandle nodeHandle;

  //Initialize different variables that are to be read from the parameter file.
  std::string broker = "localhost";
  int brokerPort = 1883;

  nodeHandle.getParam("/mqttNavdataReceiver/mqttBrokerPort", brokerPort);
  ros::param::get("/mqttNavdataReceiver/mqttBroker", broker);

  ROS_INFO("Connecting to %s at %d port\n",broker.c_str(), brokerPort);
  
  //Initialize the mqttBridge class instance
  class mqtt_bridge *mqttBridge;

  mqttBridge->lib_init();

  mqttBridge = new mqtt_bridge(CLIENTID.c_str(), broker.c_str(), brokerPort, nodeHandle);
  ROS_INFO("mqttBridge initialized..\n");


	mqttBridge->subscribe(NULL, "/ardrone/navdata");
  int rc;

  int loop_rate = 200;
  ros::param::get("/mqttNavdataReceiver/loop_rate",loop_rate);
  ros::Rate rate(loop_rate);

  ROS_INFO("Running with loop_rate: %d\n",loop_rate);

  //Now we have set everything up. We just need to loop around and act as the Bridge between ROS and MQTT.
  while(ros::ok()){

    //Pump all ROS callbacks. This function pushes all messages on the ROS subscribed topics and calls the appropriate callback
    //which were defined during the subscribe call.
    ros::spinOnce();

    //Pump all MQTT callbacks. This function pushes all messages on the MQTT Subscribed topics and calls the message_callback 
    //function defined for the mosquitto instance. The callback function handles different topics internally.
    rc = mqttBridge->loop();

		if(rc){
			mqttBridge->reconnect();
    }

    rate.sleep();
  }

  ROS_INFO("Disconnecting MQTT....\n");

  //Cleanup the Connection
  mqttBridge->disconnect();

  //Cleanup the Mosquitto Library.
  mqttBridge->lib_cleanup();

  return 0;
}

