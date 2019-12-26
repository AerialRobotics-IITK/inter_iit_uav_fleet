#include <inter_iit_uav_fleet/callbacks.h>
#include <fstream>
#include <future>

#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <inter_iit_uav_fleet/RouterData.h>

#define echo(x) std::cout << std::setprecision(10) << x << std::endl
#define sq(X) (X)*(X)
#define numQuads 4
int numObjects=5;

std::string mavName, curr_state, names[numQuads];
inter_iit_uav_fleet::RouterData routerData[numQuads - 1];
// int entries[numQuads - 1];

bool saveGPS = true;
bool received = false;
int id=-1, pubId = -1;
std::string gpsPath = "router/gps.txt";

struct locData{
    double lat;
    double lon;
    // bool publish;
};

int entries[numQuads];
struct locData objects[5];
int lastEntry = -1;

void n1Callback(const std_msgs::Int16& msg){entries[0] = msg.data;}
void n2Callback(const std_msgs::Int16& msg){entries[1] = msg.data;}
void n3Callback(const std_msgs::Int16& msg){entries[2] = msg.data;}
void r1Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[0] = msg; }
void r2Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[1] = msg; }
void r3Callback(const inter_iit_uav_fleet::RouterData& msg){ routerData[2] = msg; }

void updateTable()
{ 
    // echo(obj_data.imageID);
    for(int i = 0; i < obj_data.object_poses.size(); i++)
    {
        bool accept = true;
        // if(verbose) echo("Processing detector values");
        for(int j = 0; j <= lastEntry; j++)
        {
            if((fabs(100000*objects[j].lat -100000*obj_data.object_poses.at(i).position.x) + fabs(100000*objects[j].lon - 100000*obj_data.object_poses.at(i).position.y)) < gps_error)
            {   
                accept = false; 
                //if(verbose) echo("Rejected " << objects[j].lat << " " << objects[j].lon);
               	if(verbose) echo("Rejection Error: " << fabs(100000*objects[j].lat -100000* obj_data.object_poses.at(i).position.x) + fabs(100000*objects[j].lon -100000*obj_data.object_poses.at(i).position.y));
        		//if(verbose) echo("Min Error for accept: " << loc_error);
	        break; 
            }
        }

        if(accept)
        {
            if(lastEntry == 3) break;
            lastEntry++;
            objects[lastEntry].lat = obj_data.object_poses.at(i).position.x;
            objects[lastEntry].lon = obj_data.object_poses.at(i).position.y;
            if(verbose) echo("Accepted(D): " << objects[lastEntry].lat << " " << " " << objects[lastEntry].lon << " at index: " << lastEntry);
        }
    }

    for(int i = 0; i < numQuads - 1; i++)
    {
        // if(verbose) echo("Processing router values");
        if(lastEntry == 3) break;
        if(lastEntry < routerData[i].id && entries[i] != -1)
        {
            lastEntry++;
            objects[lastEntry].lat = routerData[i].position.x;
            objects[lastEntry].lon = routerData[i].position.y;
            if(verbose) echo("Accepted(R): " << objects[lastEntry].lat << " " << " " << objects[lastEntry].lon << " at index: " << lastEntry);
        }
    }
}

void updateRouters(ros::Publisher *pub)
{
    inter_iit_uav_fleet::RouterData msg;

    if((lastEntry > entries[0] && entries[0] != 3)|| (lastEntry > entries[1] && entries[1] != 3) || (lastEntry > entries[2] && entries[2] != 3))
    {
        int i = lastEntry - entries[0], j = lastEntry - entries[1], l = lastEntry - entries[2];
        for(int k = 0; k < i; k++)
        {
            msg.id = lastEntry - k;
            msg.position.x = objects[lastEntry - k].lat;
            msg.position.y = objects[lastEntry - k].lon; 
            msg.position.z = 0;
            pub->publish(msg);
            if(verbose) echo("Published: " << objects[lastEntry-k].lat << " " << " " << objects[lastEntry-k].lon << " from index: " << lastEntry-k);
        }
        for(int k = 0; k < j; k++)
        {
            msg.id = lastEntry - k;
            msg.position.x = objects[lastEntry - k].lat;
            msg.position.y = objects[lastEntry - k].lon; 
            msg.position.z = 0;
            pub->publish(msg);
            if(verbose) echo("Published: " << objects[lastEntry-k].lat << " " << " " << objects[lastEntry-k].lon << " from index: " << lastEntry-k);
        }
        for(int k = 0; k < l; k++)
        {
            msg.id = lastEntry - k;
            msg.position.x = objects[lastEntry - k].lat;
            msg.position.y = objects[lastEntry - k].lon; 
            msg.position.z = 0;
            pub->publish(msg);
            if(verbose) echo("Published: " << objects[lastEntry-k].lat << " " << " " << objects[lastEntry-k].lon << " from index: " << lastEntry-k);
        }
    }
}

void saveData()
{
    std::ofstream file;
    file.open(gpsPath);
    file << std::to_string(lastEntry + 1) << "\n";
    for (int i = 0; i < lastEntry + 1; i++) file << std::setprecision(10) << std::to_string(objects[i].lat) + \
     " " + std::to_string(objects[i].lon) + "\n";
    file.close();
    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "router");
    ros::NodeHandle nh, ph("~");

    ros::Rate loopRate(10);
    nh.getParam("verbose", verbose);
    nh.getParam("mav_name", mavName);
    nh.getParam("gps_error",gps_error);

    nh.getParam("names/A", names[0]);
    nh.getParam("names/B", names[1]);
    nh.getParam("names/C", names[2]);    
    nh.getParam("names/G", names[3]);
    nh.getParam("totalObjects", totalObjects);
    
    for(int i=0; i<numQuads; i++)  if(mavName == names[i]) id = i+1;
    if(verbose) echo("Quad number:" << id);

    ros::NodeHandle routers[numQuads] = {"/" + names[id-1], "/" + names[(id)%numQuads], "/" + names[(id+1)%numQuads], "/" + names[(id+2)%numQuads]};
    
    ros::Subscriber objSub = routers[0].subscribe("objects", 10, obj_cb_);
    ros::Subscriber dsubs[numQuads - 1] = {routers[1].subscribe("router/data", 10, r1Callback), routers[2].subscribe("router/data", 10, r2Callback), routers[3].subscribe("router/data", 10, r3Callback)};
    ros::Subscriber nsubs[numQuads - 1] = {routers[1].subscribe("router/num", 10, n1Callback), routers[2].subscribe("router/num", 10, n2Callback), routers[3].subscribe("router/num", 10, n3Callback)};

    ros::Publisher routerPub = ph.advertise<inter_iit_uav_fleet::RouterData>("data", 10);
    ros::Publisher numberPub = ph.advertise<std_msgs::Int16>("num", 10);

    ros::ServiceClient terminator = nh.serviceClient<std_srvs::SetBool>("stop");

    dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig> cfg_server;
    dynamic_reconfigure::Server<inter_iit_uav_fleet::reconfigConfig>::CallbackType call_f = boost::bind(&cfgCallback, _1, _2);
    cfg_server.setCallback(call_f);

    std_msgs::Int16 msg;
    entries[0] = entries[1] = entries[2] = -1;
    
    if(verbose) echo("Objects to find: " << totalObjects);
    while(ros::ok())
    {
        ros::spinOnce();
        msg.data = lastEntry;
        numberPub.publish(msg);
        updateTable();
        updateRouters(&routerPub);
        saveData();
        if(lastEntry + 1 == totalObjects)  
	    {
    		if(verbose) echo("Got " << totalObjects << " objects");
    		//std_srvs::SetBool srv; srv.request.data = false;
		    //if(verbose) echo("Calling service stop");
    		//while(!srv.response.success) 
		    //{
			//    if(verbose) echo("Waiting for response");
			//    terminator.call(srv);
		    //}
            //if(verbose) echo("Service called succesfully");
		    while(ros::ok())
	        {
			    ros::spinOnce();
			    msg.data = lastEntry;
			    numberPub.publish(msg);
			    updateRouters(&routerPub);
			    loopRate.sleep();
		    }
	    }
        loopRate.sleep();
    }

    return 0;
}
