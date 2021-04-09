using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Timers;
using EventArgsLibrary;
using System.Threading;

namespace PololuControllerNs
{
    public class PololuController
    {
        Thread t1;

        public enum command
        {
            SetTarget = 0x84,
        }
        public PololuController()
        {

            t1 = new Thread(ForwardUart);
            t1.Start();
        }



        void SetTurbineSpeed(byte Channel, UInt16 Duty)
        {
            byte[] frame = new byte[4];
            Duty *= 4;
            frame[0] = (byte)command.SetTarget;
            frame[1] = Channel;
            frame[2] = (byte)(Duty & 0x7F);
            frame[3] = (byte)((Duty >> 7) & 0x7F);
            UartForward(frame);
        }

        void GrabCup()
        {
            SetTurbineSpeed(3, 1200);
            Thread.Sleep(1000);
            SetTurbineSpeed(3, 1100);
        }
         
        void ForwardUart()
        {
            byte[] channels = { 0, 1, 2, 3, 4 };
            byte servoMotor = 5;
            ushort stateTop = 1785;
            ushort stateBot = 2000;
            EnableAllMotors(channels);
            EnableOneMotor(servoMotor);
            //GrabCup();
            //SetTurbineSpeed(3, 1050);
            //Thread.Sleep(3000);
            DisableAllMotors(channels);
            /*
            SetTurbineSpeed(servoMotor, stateTop);
            Thread.Sleep(3000);
            SetTurbineSpeed(servoMotor, stateBot);
            Thread.Sleep(3000);*/

            //Speed for first grab at 13 cm from the ground verticaly :+
            // 0 = 1120
            // 1 = 
            // 2 = 1050
            // 3 = 1065
            // 4 = 

            /*EnableAllMotors(channels);
            foreach (byte c in channels) { SetTurbineSpeed(c, 1040); }
            Thread.Sleep(5000);
            DisableAllMotors(channels);*/
            //ushort speed = 1020;
            //EnableAllMotors(channels);
            //SetTurbineSpeed(3, 1100);
            //Thread.Sleep(5000);
            /*
            while (true)
            {
                if(speed < 1280)
                {
                    foreach (byte c in channels) { SetTurbineSpeed(c, speed); }
                    speed += 5;
                    Console.WriteLine(speed);
                    Thread.Sleep(4000);
                }
                else
                {
                    break;
                }
            }*/
            //
        }

        //Control all motors at the same time
        void EnableAllMotors(byte[] channels)
        {
            foreach(byte c in channels) { SetTurbineSpeed(c, 0); }
            Thread.Sleep(400);

            foreach (byte c in channels) { SetTurbineSpeed(c, 64); }
            Thread.Sleep(1000);
        }
        void DisableAllMotors(byte[] channels)
        {
            foreach (byte c in channels) { SetTurbineSpeed(c, 0); }
            Thread.Sleep(400);
        }

        //Control only one motor
        void EnableOneMotor(byte channel)
        {   
            if (channel == 5 )
            {
                SetTurbineSpeed(channel, 0);
                Thread.Sleep(400);
                SetTurbineSpeed(channel, 2000);
                Thread.Sleep(1000);
            }
            else
            {
                SetTurbineSpeed(channel, 0);
                Thread.Sleep(400);
                SetTurbineSpeed(channel, 64);
                Thread.Sleep(1000);
            }

        }
        void DisableOneMotor(byte channel)
        {
            SetTurbineSpeed(channel, 0);
            Thread.Sleep(400);
        }

        void BadApple()
        {
            /*
            SetTurbineSpeed(3, 1020);
            Thread.Sleep(400);
            SetTurbineSpeed(3, 10);
            Thread.Sleep(200);
            SetTurbineSpeed(2, 1025);
            Thread.Sleep(400);
            SetTurbineSpeed(2, 10);
            Thread.Sleep(200);
            SetTurbineSpeed(0, 1060);
            Thread.Sleep(400);
            SetTurbineSpeed(0, 10);
            Thread.Sleep(200);*/
        }

        public event EventHandler<DataReceivedArgs> UartForwardEvent;
        public virtual void UartForward(byte[] data)
        {
            UartForwardEvent?.Invoke(this, new DataReceivedArgs
            {
                Data = data
            });
        }
    }
}
