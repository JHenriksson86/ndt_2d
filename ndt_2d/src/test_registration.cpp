// STL
#include <iostream>
#include <vector>
#include <string>
#include <fstream>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

// NDT
#include "ndt_2d_registration.h"

using namespace ndt2d;
using std::cout;
using std::endl;
using std::string;
using std::vector;

vector<double> createRange(double min_value, double max_value, unsigned int size)
{
   vector<double> range;
   if(min_value > max_value)
      return range;
   else if(min_value == max_value)
   {
      range.push_back(min_value);
      return range;
   }
   else if(size == 0)
   {
      range.push_back(0.0);
      return range;
   }

   double step = (max_value - min_value) / (double)(size-1);
   for(unsigned int i = 0; i < size; i++)
   {
      range.push_back(min_value + step*((double)i));
   }
   return range;
}

bool saveRegistrationResult(const TransformationVector& actual,
   const TransformationVector& result, double time_taken, double iterations,
   const std::string& filename, const std::string& path = "")
{
   std::string output_filename = path + "/" + filename + ".result";
   ROS_INFO("Saving data to %s", output_filename.c_str());

   std::ofstream output;
   output.open(output_filename.c_str(), std::ofstream::out | std::ofstream::app);

   TransformationVector diff = result.calculateAbsoluteDifference(actual);
   double squared_norm_diff = diff.getParameterVector().squaredNorm();

   output << time_taken << ";" << iterations << ";" 
      << actual.to_string() << ";" << result.to_string() << ";" 
      << diff.to_string() << ";"  <<  squared_norm_diff
      << "\n";

   output.close();
   return true;
}

int main(int argc, char **argv)
{
   ros::init(argc, argv, "ndt_2d_test_registration_node");
   ros::NodeHandle nh;

   ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
   ros::console::notifyLoggerLevelsChanged();

   //Loading parameters
   // Costfunction either 0 for standard or 1 for step
   int cost_function = 0;
   ros::param::get("/ndt_2d_test_registration_node/cost_function", cost_function);
   if(cost_function == 1)
      cout << "Cost function = Step\n";
   else{
      cost_function = 0;
      cout << "Cost function = Standard\n";
   }

   int cell_count = 20;
   ros::param::get("/ndt_2d_test_registration_node/cell_count", cell_count);

   double cell_size = 1.0;
   ros::param::get("/ndt_2d_test_registration_node/cell_size", cell_size);

   // Min values of the test range
   double x_min = 0.0;
   ros::param::get("/ndt_2d_test_registration_node/x_min", x_min);

   double y_min = 0.0;
   ros::param::get("/ndt_2d_test_registration_node/y_min", y_min);

   double rz_min = 0.0;
   ros::param::get("/ndt_2d_test_registration_node/rz_min", rz_min);

   // Max values of the test range
   double x_max = 0.0;
   ros::param::get("/ndt_2d_test_registration_node/x_max", x_max);

   double y_max = 0.0;
   ros::param::get("/ndt_2d_test_registration_node/y_max", y_max);

   double rz_max = 0.0;
   ros::param::get("/ndt_2d_test_registration_node/rz_max", rz_max);

   // Size of the test range
   int range_size_x = 0;
   ros::param::get("/ndt_2d_test_registration_node/range_size_x", range_size_x);

   int range_size_y = 0;
   ros::param::get("/ndt_2d_test_registration_node/range_size_y", range_size_y);

   int range_size_rz = 0;
   ros::param::get("/ndt_2d_test_registration_node/range_size_rz", range_size_rz);

   // Scan file path
   string scan_file_path = "";
   ros::param::get("/ndt_2d_test_registration_node/scan_file_path", scan_file_path);

   // Result file path
   string result_path = "";
   ros::param::get("/ndt_2d_test_registration_node/result_path", result_path);

   // Result filename
   string result_filename = "";
   ros::param::get("/ndt_2d_test_registration_node/result_filename", result_filename);
   result_filename = result_filename + "-cc" + std::to_string(cell_count) 
      + "-cs" + std::to_string(cell_size);

   // Loading laserscans
   cout << "Loading fixed scan\n";
   LaserScan fixed_scan;
   fixed_scan.loadLaserScan("test", scan_file_path);
   cout << "Loaded " << fixed_scan.size() << " points\n\n";

   cout << "Loading movable scan." << endl;
   LaserScan movable_scan;
   movable_scan.loadLaserScan("test", scan_file_path);
   cout << "Loaded " << movable_scan.size() << " points\n\n";

   // NDT Conversion on fixed laser_scan
   cout << "Converting fixed scan to NDT.\n";
   cout << "Cell count = " << cell_count << ", Cell size = " << cell_size << "\n";
   NDTMap fixed_ndt(fixed_scan, cell_count, cell_count, cell_size);
   fixed_ndt.calculateGaussians();
   cout << "Conversion done!\n\n";

   // Create arrays with ranges
   vector<double> range_x = createRange(x_min, x_max, range_size_x);
   vector<double> range_y = createRange(y_min, y_max, range_size_y);
   vector<double> range_rz = createRange(rz_min, rz_max, range_size_rz);

   // Create registration instance
   NDTRegistrationCeres registration((CostFunction)cost_function);
   cout << registration.options_to_string();
   
   double start =ros::Time::now().toSec();
   for(unsigned int x = 0; x < range_x.size(); x++)
   {
      for(unsigned int y = 0; y < range_y.size(); y++)
      {
         for(unsigned int rz = 0; rz < range_rz.size(); rz++)
         {
            cout << "x = " << range_x[x]
               << ", y = " << range_y[y]
               << ", rz = " << range_rz[rz] << "\n";

            // Transforming movable laserscan
            TransformationVector transformation(range_x[x], range_y[y], range_rz[rz]);
            LaserScan transformed_scan = movable_scan.getTransformed(transformation);

            // NDT Conversion
            NDTMap movable_ndt(transformed_scan, cell_count, cell_count, cell_size);
            movable_ndt.calculateGaussians();

            // Registration
            registration.setMaps(&fixed_ndt, &movable_ndt);
            registration.optimize();

            saveRegistrationResult(transformation, registration.getTransformation(),
               registration.getTimeTaken(), registration.getNumberOfIterations(),
               result_filename, result_path);
         }
      }
   }
   double end = ros::Time::now().toSec();
   int number_of_registrations = range_x.size()*range_y.size()*range_rz.size();
   double duration = end - start;
   double avg_time = duration / (double)number_of_registrations;

   cout << "Number of registrations: " << number_of_registrations << "\n";
   cout << "Total time taken: " << duration << "s\n";
   cout << "Average time per reg: " << avg_time << "s\n";
   
   return 0;
}