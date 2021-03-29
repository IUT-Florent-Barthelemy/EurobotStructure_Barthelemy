using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Timers;
using EventArgsLibrary;
using Utilities;

namespace RobotSimulator
{
    public class RobotSim
    {
        System.Timers.Timer OdomTimer;
        int robotId;

        public RobotSim(int robotId,bool enable, double frequencyHz)
        {
            if(enable)
            {
                this.robotId = robotId;
                OdomTimer = new Timer();
                OdomTimer.Interval = 1 / frequencyHz;
                OdomTimer.Elapsed += OdomTimer_Elapsed;
                OdomTimer.Start();
            }
        }

        private void OdomTimer_Elapsed(object sender, ElapsedEventArgs e)
        {
            OnPositionEmulated();
        }

        public event EventHandler<LocationArgs> OnPositionEmulatedEvent;
        public virtual void OnPositionEmulated()
        {
            OnPositionEmulatedEvent?.Invoke(this, new LocationArgs
            {
                RobotId = robotId,
                Location = new Location(0, 0, 0, 0, 0, 0)
            });
        }

    }
}
