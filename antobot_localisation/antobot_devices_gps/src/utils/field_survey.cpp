
/* script to survey the farm. 

Copyright (c) 2023, ANTOBOT LTD.
All rights reserved

Press key 's' to save the coordinates of the current pos; it saves average of 80 recent Lat, lon coordinates. 
Make sure to clean the buffer by pressing 'c' before saving the GPS coordinates for better accuracy.
It listens to the ros topic /am_gps_urcu

Contact: Zhuang Zhou
email:  zhuang.zhou@antobot.ai
*/
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <sensor_msgs/NavSatFix.h>
#include <vector>
#include <fstream>
#include <iomanip>
#include <time.h>


std::ofstream gps_file;
int buffer = 80,count = 0; 

double lat,lon;
int key_press = 0;
int gps_status = 0;


//std::vector<sensor_msgs::NavSatFix> gps_data;
std::vector<double> gps_lat;
std::vector<double> gps_lon;

//void write();

/*########################################################################################*/
//GPS callback function to avergae out 80 recent GPS coordinates and to write to an xml file

void gpsCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    //std::cout<<"In call back function"<<std::endl;
    char option = '\0';
    int id = 0;
    gps_status = msg->status.status;
    
    bool is_sim = true;
    if (is_sim)
        gps_status = 3;

     if (key_press == 2)
     {
        //ret = 0;
        std::cout<<"Cleaning the buffer"<<std::endl;
        gps_lat.clear();
        gps_lon.clear();
        key_press = 0;
     }
    else if (key_press == 1 and gps_status == 3)
    {
     gps_lat.push_back(msg->latitude);    
     gps_lon.push_back(msg->longitude);

     //std::cout << "Waiting to satisfy the buffer length" <<std::endl;

     if (gps_lon.size() >= buffer)
     {
        // count +=1;

        //std::cout<<"Buffer length satisfied"<<std::endl;
        //std::cout<<"Number of gps coordinates received"<<gps_lat.size()<<std::endl;
        double sum_lat = 0.0;
        double sum_lon = 0.0;
    
        int len = 0;
        for (int i = gps_lat.size() - buffer; i < gps_lat.size(); i++)
        
        //for (int i = 0; i < buffer; i++)
            {
                // std::cout << "Data: " << gps_data.size() - buffer << std::endl;
                
                sum_lat +=gps_lat[i] ;
                sum_lon +=gps_lon[i];
                len+=1;
            }
            //std::cout<<"Aveg:"<<sum/len<<std::endl;
            //std::cout<<"size of topic data"<<gps_data.size()<<std::endl;
        
        
        
        // script to check if the user has the node id; if so then ask the user to input it and save it along with the GPS coordinates in the xml file
        std::cout<<"Do you know the node ID? y/n (Use lowercase)"<<std::endl;
        std::cin>> option;

        
        // if user knows the node id;  ask the user to enter the node id
        if (option == 'y')
        {
            std::cout<<std::endl<<"enter the node id"<<std::endl;
            std::cin>>id;
        }
        else if (option =='n')
        {
            id = count;
            count +=1;
        }
        else
        {
            id = -1;
            std::cout<<"No valid node ID; value is not written to the file. Try again..!!";
        }
        gps_file<< std::fixed << std::setprecision(17);
        gps_file << "\t<node id=\"" << id <<"\" lat=\"" << sum_lat/len << "\" lon=\" "<<sum_lon/len<<"\"/>"<<std::endl;
        std::cout<<"Value written to file lat:"<< std::fixed << std::setprecision(17)<< sum_lat/len <<" , long: "<<sum_lon/len<<std::endl;

        gps_lat.clear();
        gps_lon.clear();
        //std::cout<<"buffer is cleared and the size of the vector: "<<gps_lat.size();
        key_press = 0;
        // std::cout<<"key press value changed to 0: "<< key_press;
        
     }
    }


}
/*########################################################################################*/
//function to check the key press
//Always press 'c' to clear the buffer and then press 's' to save the coordinates
int KeyPress(char key)	
{	
    // bool key_press = 0;	
    if (key == 's')	
    {	
        key_press = 1;	
        std::cout<<"Save way point request is received...!!"<<std::endl;	
        while (gps_lat.size() < buffer and key_press == 1)	
        {	
            ros::spinOnce();	
        }	
        // std::cout<<"key press value changed to  1:"<< key_press;	
        	
    }	
    else if (key == 'c')	
    {	
        key_press = 2;	
        gps_lat.clear();	
        gps_lon.clear();
        	
        // std::cout<<"key press value changed to 2: "<< key_press;	
        	
    }
    else if (key == 'f')
    {
        key_press = 3;
    }	
    else	
    {	
        key_press = 0;	
    }	
    return key_press;	
}

void create_file_structure()
{
    gps_file<<"<?xml version=\"1.0\" ?>" << std::endl;
    gps_file<<"<antonav version=\"0.1\" generator=\"surveyScript\">" << std::endl;
}

void close_file_structure()
{
    gps_file<<"</antonav>";
}

/*########################################################################################*/
int main(int argc, char** argv)
{
    ros::init(argc, argv, "am_survey");
    
    ros::NodeHandle n;
    time_t now = time(0);
    struct tm timeinfo;
    localtime_r(&now, &timeinfo);
    std::stringstream timestamp;
    timestamp << "survey_" << timeinfo.tm_year + 1900 << timeinfo.tm_mon + 1 << timeinfo.tm_mday << "_" << timeinfo.tm_hour << timeinfo.tm_min << timeinfo.tm_sec;
    std::string timestamp_s;
    timestamp >> timestamp_s;

    //const std::string path = "/root/catkin_ws/src/AntoMove/antobot_move_navigation/antobot_move_cartograph/src/XML data/" + timestamp_s + ".antonav.xml";
    const std::string path = "/home/antobot/catkin_ws/src/AntoSites/antobot_sites/xml_data/" + timestamp_s + ".antonav.xml";
    //const std::string path = "/home/antobot/catkin_ws/src/AntoSites/antobot_sites/xml_data/courtyardTest.antonav.xml";
    gps_file.open(path);
    //gps_file.open("new_xml.txt");
    create_file_structure();

    ros::Subscriber sub = n.subscribe("/antobot_f9p_usb", 10000, gpsCallback);

    struct termios old_tio, new_tio;
    tcgetattr(STDIN_FILENO, &old_tio);
    new_tio = old_tio;
    new_tio.c_lflag &= ~ICANON;
    tcsetattr(STDIN_FILENO, TCSANOW, &new_tio);


    while (ros::ok())
    {

        std::cout<<"value of key_press "<<key_press << std::endl;
        std::cout<<"Clean the buffer 'c' before pressing 's' (use lower case)"<<std::endl;

        char key = getchar();
        key_press = KeyPress (key);
        if (key_press == 3)
        {
            close_file_structure();
            break;
        }
        ros::spinOnce();
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_tio);

    return 0;
}
