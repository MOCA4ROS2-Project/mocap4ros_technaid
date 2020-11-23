using System;
using System.Reflection;
using System.Runtime;
using System.Runtime.InteropServices;
using System.Threading;

using SDK_MCS_Studio2;
using MatrixLibrary;
using ROS2;
using ROS2.Utils;
using System.Numerics;


namespace ConsoleApplication {
  public class ImuDriver {

    private static cMCSHub captureDevice;

    private static cMCSSensor sensor;

    private static cMCSImu imu;

    private const int dataLength = 100;

    static int numData = 9;

    private static UInt16[,] dataReceived = new UInt16[9, dataLength];

    private static IPublisher<sensor_msgs.msg.Imu> imu_pub;

    static void NewFrameReceived(object sender, EventArgs e)
    {
      UInt16[] tempIntData = new UInt16[9];
      int numFrame = captureDevice.getNumFrames() - 1;
            
      tempIntData[0] = sensor.getDigitalFrame(numFrame).Acce.X;
      tempIntData[1] = sensor.getDigitalFrame(numFrame).Acce.Y;
      tempIntData[2] = sensor.getDigitalFrame(numFrame).Acce.Z;
      tempIntData[3] = sensor.getDigitalFrame(numFrame).Gyro.X;
      tempIntData[4] = sensor.getDigitalFrame(numFrame).Gyro.Y;
      tempIntData[5] = sensor.getDigitalFrame(numFrame).Gyro.Z;
      tempIntData[6] = sensor.getDigitalFrame(numFrame).Magn.X;
      tempIntData[7] = sensor.getDigitalFrame(numFrame).Magn.Y;
      tempIntData[8] = sensor.getDigitalFrame(numFrame).Magn.Z;
            
      for (int j = 0; j < numData; ++j)
      {
        dataReceived[j, dataLength - 1] = tempIntData[j];
        Array.Copy(dataReceived, dataLength * j + 1, dataReceived, dataLength * j, dataLength - 1);
      }

      for (int col = 0; col < dataLength; col++)
      {
        sensor_msgs.msg.Imu imu_data_msg = new sensor_msgs.msg.Imu();

        imu_data_msg.Header.Frame_id = sensor.getIMUInfo().Label;
        imu_data_msg.Linear_acceleration.X = dataReceived[col, 0];
        imu_data_msg.Linear_acceleration.Y = dataReceived[col, 1];
        imu_data_msg.Linear_acceleration.Z = dataReceived[col, 2];
        imu_data_msg.Angular_velocity.X = dataReceived[col, 3];
        imu_data_msg.Angular_velocity.Y = dataReceived[col, 4];
        imu_data_msg.Angular_velocity.Z = dataReceived[col, 5];
        Quaternion q = Quaternion.CreateFromYawPitchRoll(dataReceived[col, 6], dataReceived[col, 7], dataReceived[col, 8]);
        imu_data_msg.Orientation.X = q.X;
        imu_data_msg.Orientation.Y = q.Y;
        imu_data_msg.Orientation.Z = q.Z;
        imu_data_msg.Orientation.W = q.W;

        imu_pub.Publish(imu_data_msg);
      
      }
    }



    static public void ImusConfiguration()
    {
      Console.WriteLine("Sensor configuration:\n\n"+
        "Firmware version:" + sensor.getIMUInfo().firmwareVersion + "\n"+
        "Frequency:" + sensor.getIMUInfo().Frequency.ToString() + 
        "\n"+"NID:" + sensor.getIMUInfo().NID.ToString() + 
        "\n"+ "Serial number:" + sensor.getIMUInfo().serialNumber);
        
    }

    static public bool DisconnectImu()
    {
      captureDevice.Disconnect();

      captureDevice.DigitalFrameReceived -= new EventHandler(NewFrameReceived);

      if (captureDevice.IsConnect())
      {
        Console.WriteLine ("Imu camera is connected.");
        return false;
      }

      captureDevice.StopCapture();

      Console.WriteLine ("Imu camera is disconnected.");
      return true;
    }

    static public void InitImuCapture()
    {
      imu.setFrequency(100);
      imu.DigitalFrameReceived += new EventHandler(NewFrameReceived);
      imu.StartCapture(CAPTURE_INFORMATION.DigitalData, false, false, false);
    }

    static public bool ConnectImu()
    {
      captureDevice = new cMCSHub();
      sensor = captureDevice.Sensors[0];
      captureDevice.Connect();

      if (captureDevice.IsConnect() && captureDevice.AutoDetectPort() == FUNCTION_RESULT.FunctionOK)
      {
        Console.WriteLine ("Imu camera is connected.");
        
        return captureDevice.IsConnect();
      }

      Console.WriteLine ("Imu camera is disconnected.");
      return captureDevice.IsConnect();
    }

    public static void Main (string[] args) {

      if (ConnectImu()){
        RCLdotnet.Init ();

        INode node = RCLdotnet.CreateNode ("imus_driver_node");

        imu_pub = node.CreatePublisher<sensor_msgs.msg.Imu> ("/imu");
        
        InitImuCapture();
      }
    }
  }
}