﻿using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace Positioning2WheelsNS
{
    public class Positioning2Wheels
    {
        Location ActualLocation;
        double Tech_Sec = 1 / 50f;
        int robotID;



        public Positioning2Wheels(int robotID, double x = 0, double y = 0, double theta = 0)
        {
            this.robotID = robotID;
            ActualLocation = new Location(x, y, theta, 0, 0, 0);
        }

        public void OnOdometryRobotSpeedReceived(object sender, PolarSpeedArgs e)
        {
            ActualLocation.Theta += ActualLocation.Vtheta * Tech_Sec;

            ActualLocation.Vx = e.Vx * Math.Cos(ActualLocation.Theta);
            ActualLocation.Vy = e.Vx * Math.Sin(ActualLocation.Theta);
            ActualLocation.Vtheta = e.Vtheta;

            ActualLocation.X += ActualLocation.Vx * Tech_Sec;
            ActualLocation.Y += ActualLocation.Vy * Tech_Sec;
            

            //Console.WriteLine("Location Vx = {0}", ActualLocation.Vx);

            OnCalculatedLocation(robotID, ActualLocation);
        }

        //Output events
        public event EventHandler<LocationArgs> OnCalculatedLocationEvent;
        public virtual void OnCalculatedLocation(int id, Location locationRefTerrain)
        {
            var handler = OnCalculatedLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = locationRefTerrain });
            }
        }
    }
}
