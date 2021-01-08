/*
 * Introduces the following
 *      1. Get the parameters on the Parameter Server
 *      2. Create parameters on the parameter server
 *      3. Deleting parameters on the parameter server
 */

// Include the ROS header file
#include <ros/ros.h>
// For XMLRPS
#include <xmlrpcpp/XmlRpcUtil.h>

using namespace std;

int main(int argc, char **argv) {
    // Initialize the node
    ros::init(argc, argv, "simple_cpp_parameter_node");
    // Create a NodeHandle which will be used to access the parameter server (master)
    ros::NodeHandle nh;
    
    // Retrieve a list of parameters running on the parameter server
    /*
     * Parameters are essentially (key, value) pairs. This function can retrieve a list of keys (as a string vector)
     * 
     * Link (getParamNames function): https://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a00d8052ccffb3ae130d6bfb1214c9ce0
     */
    vector<string> param_names;
    nh.getParamNames(param_names);  // Get a list of parameter names into the variable
    
    // Print out the names
    ROS_INFO("Found the following parameters");
    for (int i = 0; i < param_names.size(); i++) {
        
        // Get the value of the parameter
        /*
         * Use the getParam method to retrieve the value. The method returns if the parsing was successful (all string parsings must be successful)
         * 
         * Link (getParam function): https://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#afb8dbc451e3c0dbc14c67438d21c9f2b
         */
        string param_value;
        bool found_val = nh.getParam(param_names.at(i), param_value);
        if (found_val) {
            ROS_INFO_STREAM("Parameter " << i + 1 
                            << " = '" << param_names.at(i) 
                            << "' -> '" << param_value << "'"
                           );
        } else {
            ROS_WARN_STREAM("Parameter " << i + 1 << " = '" << param_names.at(i) << "' -> [NOT STRING]");
        }
    }
    
    // Get a specific parameter (here "num_param")
    /*
     * It's better to search for a parameter before actually retrieving it
     * 
     * Link (hasParam function): https://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a4c2c87e9deb0cbbeae2f96d7cfaefae0
     */
    string src_param = "num_param";
    double dparam_value;
    if (nh.hasParam(src_param)) {
        // Get the parameter value into double
        if (nh.getParam(src_param, dparam_value)) {
            ROS_INFO_STREAM("Parameter '" << src_param << "' -> " << dparam_value);
        } else {
            ROS_WARN_STREAM("Parameter value for '" << src_param << "' could not be parsed to double");
        }
    } else {
        ROS_WARN_STREAM("Parameter '" << src_param << "' could not be found");
    }
    
    // Get a parameter as an XML tree
    src_param = "parent_param";
    string param_value;
    if (nh.hasParam(src_param)) {   // See if value exists
        if (nh.getParam(src_param, param_value)) {  // See if it fits in a string
            ROS_INFO_STREAM("Parameter '" << src_param << "' -> '" << param_value << "'");
        } else {    // Read the value through XML
            ROS_WARN_STREAM("Parameter '" << src_param << "' could not be parsed as string");
            // Try getting through XMLRPC
            /*
             * XMLRPC reference: https://docs.ros.org/en/api/xmlrpcpp/html/classXmlRpc_1_1XmlRpcUtil.html
             */
            XmlRpc::XmlRpcValue val;
            if (nh.getParam(src_param, val)) {
                // val has everything in XML format, it can be parsed using some XML parser
                ROS_INFO_STREAM("Parameter '" << src_param << "' -> [XML] '"
                                << val.toXml()
                                << "'");
            } else {
                ROS_WARN_STREAM("Could not get XMLRPC for parameter '" << src_param << "'");
            }
            // As we know "/parent_param/child_param/param_1" is a parameter, we try to get the value
            src_param = "/parent_param/child_param/param_1";
            if (nh.getParam(src_param, param_value)) {
                ROS_INFO_STREAM("Parameter '" << src_param << "' -> '"
                                << param_value << "'"
                );
            } else {
                ROS_WARN_STREAM("Parameter '" << src_param << "' could not be found");
            }
        }
    } else {
        ROS_WARN_STREAM("Parameter '" << src_param << "' could not be found");
    }
    
    // Get parameter value as an array
    src_param = "list_param";
    if (nh.hasParam(src_param)) {
        vector<string> list_param_value;
        if (nh.getParam(src_param, list_param_value)) {
            string val = "";
            for (const string &str: list_param_value) {
                val += str;
                val += "; ";
            }
            val = val.substr(0, val.length()-2);    // Remove the '; ' in the end (last one)
            ROS_INFO_STREAM("Parameter '" << src_param << "' -> (; separated) '" << val << "'");
        } else {
            ROS_WARN_STREAM("Parameter '" << src_param << "' could not be parsed to vector<string>");
        }
    } else {
        ROS_WARN_STREAM("Parameter '" << src_param << "' could not be found");
    }
    
    // Create a parameter on the parameter server
    /*
     * Use the setParam function to upload a key, value parameter to the server
     * 
     * Link (setParam function): https://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a1eafe9844a6802412e04054bc294fc58
     */
    string param_name = "custom_cpp_parameter";
    param_value = "Custom C++ Parameter";
    nh.setParam(param_name, param_value);
    ROS_INFO_STREAM("Created parameter '" << param_name << "'");
    
    // Delete a parameter on the parameter server
    /*
     * Use the deleteParam function to delete a parameter
     * 
     * Link (deleteParam function): https://docs.ros.org/en/latest/api/roscpp/html/classros_1_1NodeHandle.html#a7ae0bb64c6092661b63741d90d6438b3
     */
    param_name = "junk_parameter";
    if (nh.deleteParam(param_name)) {
        ROS_INFO_STREAM("Successfully deleted parameter '" << param_name << "'");
    } else {
        ROS_WARN_STREAM("Parameter '" << param_name << "' could not be deleted (maybe not existing?)");
    }
    
    // Go into indefinite spin
    ros::spin();
    
    return 0;
}
