using EventArgsLibrary;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Utilities;

namespace TrajectoryGeneratorNonHolonomeNS
{
    public class TrajectoryGeneratorNonHolonome
    {
        int robotId;

        double samplingPeriod = 1/50.0;

        Location currentLocationRefTerrain;
        Location wayPointLocation;
        Location ghostLocationRefTerrain;

        double accelLineaire, accelAngulaire;
        double vitesseLineaireMax, vitesseAngulaireMax;

        AsservissementPID PID_Position_Lineaire;
        AsservissementPID PID_Position_Angulaire;


        enum States
        {
            idle,
            rotation_1,
            linear,
            rotation2,
            fastDeceleration
        }

        States state = States.idle;

        double deadZoneLin = 0.001;
        double deadZoneTheta = 0.001;



        public TrajectoryGeneratorNonHolonome(int id)
        {
            robotId = id;
            InitRobotPosition(0, 0, 0);
            InitPositionPID();

            //********************************************************SET NEW POSITION



            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 0.5; //en m.s-2
            accelAngulaire = 1 * Math.PI * 1.0; //en rad.s-2

            vitesseLineaireMax = 1.5; //en m.s-1               
            vitesseAngulaireMax = 3 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(20.0, 10.0, 0, 100, 100, 1);
            PID_Position_Angulaire = new AsservissementPID(20.0, 10.0, 0, 5 * Math.PI, 5 * Math.PI, Math.PI);
        }

        public void InitRobotPosition(double x, double y, double theta)
        {
            Location old_currectLocation = currentLocationRefTerrain;
            currentLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            wayPointLocation = new Location(x, y, theta, 0, 0, 0);
            ghostLocationRefTerrain = new Location(x, y, theta, 0, 0, 0);
            PIDPositionReset();
        }

        public void SetDestination(object sender, PositionArgs destination)
        {
            wayPointLocation.X = destination.X;
            wayPointLocation.Y = destination.Y;
            state = States.rotation_1;
        }

        public void OnPhysicalPositionReceived(object sender, LocationArgs e)
        {
            if (robotId == e.RobotId)
            {
                currentLocationRefTerrain = e.Location;
                CalculateGhostPosition();
                PIDPosition();
            }
        }


        bool old_param = false;
        bool param = false;
        bool depassement = false;

        void CalculateGhostPosition()
        {
            //A remplir


            switch(state)
            {
                case States.idle:
                    ghostLocationRefTerrain.Vlin = 0;
                    ghostLocationRefTerrain.Vtheta = 0;
                    break;

                case States.rotation_1:
                    if(ghostLocationRefTerrain.Vlin >= 0.01)
                    {
                        state = States.fastDeceleration;
                        break;
                    }
                        
                    ghostLocationRefTerrain.Vlin = 0;
                    double ThetaCorrect = Math.Atan2(wayPointLocation.Y - ghostLocationRefTerrain.Y, wayPointLocation.X - ghostLocationRefTerrain.X);
                    double ThetaStopDistance = Math.Pow(ghostLocationRefTerrain.Vtheta, 2) / (2 * accelAngulaire);
                    double ThetaRestant = ThetaCorrect - Toolbox.ModuloByAngle(ThetaCorrect, ghostLocationRefTerrain.Theta);//ThetaCorrect - Toolbox.Modulo2PiAngleRad(ghostLocationRefTerrain.Theta);// 

                    

                    if (ThetaStopDistance < Math.Abs(ThetaRestant)) //acceleration
                    {
                        
                            if (ThetaRestant > 0)
                                ghostLocationRefTerrain.Vtheta += accelAngulaire * samplingPeriod;
                                
                            else
                                ghostLocationRefTerrain.Vtheta -= accelAngulaire * samplingPeriod;


                            ghostLocationRefTerrain.Vtheta = Toolbox.LimitToInterval(ghostLocationRefTerrain.Vtheta, -vitesseAngulaireMax, vitesseAngulaireMax);
                    }

                    else//deceleration
                    {
                        if (ghostLocationRefTerrain.Vtheta > 0)
                            ghostLocationRefTerrain.Vtheta -= accelAngulaire * samplingPeriod;

                        else
                            ghostLocationRefTerrain.Vtheta += accelAngulaire * samplingPeriod;

                        ghostLocationRefTerrain.Vtheta = Toolbox.LimitToInterval(ghostLocationRefTerrain.Vtheta, -vitesseAngulaireMax, vitesseAngulaireMax);

                    }

                    

                    if (Math.Abs(ThetaRestant) < Toolbox.DegToRad(0.2))
                    {
                        state = States.linear;
                        old_param = (ThetaCorrect > 0) ? (true) : (false); ;
                    }
                        

                    break; 

                case States.linear:
                    ghostLocationRefTerrain.Vtheta = 0;
                    PointD destinationPoint = new PointD(wayPointLocation.X, wayPointLocation.Y);
                    PointD ptSeg2 = new PointD(ghostLocationRefTerrain.X + Math.Cos(ghostLocationRefTerrain.Theta), ghostLocationRefTerrain.Y + Math.Sin(ghostLocationRefTerrain.Theta));
                    PointD ptSeg1 = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
                    double DistanceRestante = Toolbox.ProjectedDistanceOfPointOnLine(destinationPoint, ptSeg1, ptSeg2);
                    double stopDistance = Math.Pow(ghostLocationRefTerrain.Vlin, 2) / (2 * accelLineaire);

                    //polarisation de la distance
                    double Theta1 = Math.Atan2(wayPointLocation.Y - ghostLocationRefTerrain.Y, wayPointLocation.X - ghostLocationRefTerrain.X);
                    param = (Theta1 > 0) ? (true) : (false);

                    if (param != old_param)
                    {
                        Console.WriteLine("Depassement");
                        depassement = true;
                    }


                        if (DistanceRestante > stopDistance)
                            ghostLocationRefTerrain.Vlin += accelLineaire * samplingPeriod;


                        if (DistanceRestante < stopDistance)
                            ghostLocationRefTerrain.Vlin -= accelLineaire * samplingPeriod;



                    if (DistanceRestante < 0.0001)
                    {
                        state = States.idle;
                        depassement = false;
                        Console.WriteLine("Erreur: {0}", DistanceRestante);
                    }
                        
                     
                    old_param = param;
                    break;

                case States.fastDeceleration:
                    if (ghostLocationRefTerrain.Vlin > 0)
                        ghostLocationRefTerrain.Vlin -= 2*accelLineaire * samplingPeriod;
                    else
                        ghostLocationRefTerrain.Vlin += 2*accelLineaire * samplingPeriod;

                    if (ghostLocationRefTerrain.Vlin <= 0.01)
                        state = States.rotation_1;

                    break;
            }

            ghostLocationRefTerrain.Vx = ghostLocationRefTerrain.Vlin * Math.Cos(ghostLocationRefTerrain.Theta);
            ghostLocationRefTerrain.Vy = ghostLocationRefTerrain.Vlin * Math.Sin(ghostLocationRefTerrain.Theta);

            ghostLocationRefTerrain.X += ghostLocationRefTerrain.Vx * samplingPeriod;
            ghostLocationRefTerrain.Y += ghostLocationRefTerrain.Vy * samplingPeriod;            
            ghostLocationRefTerrain.Theta += ghostLocationRefTerrain.Vtheta * samplingPeriod;


            //On renvoie la position du ghost pour affichage
            OnGhostLocation(robotId, ghostLocationRefTerrain);
        }

        void PIDPosition()
        {
            //A remplir
            double vLineaireRobot=0, vAngulaireRobot=0;


            //Si tout c'est bien passé, on envoie les vitesses consigne.
            OnSpeedConsigneToRobot(robotId, (float)vLineaireRobot, (float)vAngulaireRobot);
        }

        void PIDPositionReset()
        {
            if (PID_Position_Angulaire != null && PID_Position_Lineaire != null)
            {
                PID_Position_Lineaire.ResetPID(0);
                PID_Position_Angulaire.ResetPID(0);
            }
        }


        /*************************************** Outgoing Events ************************************/

        public event EventHandler<LocationArgs> OnGhostLocationEvent;
        public virtual void OnGhostLocation(int id, Location loc)
        {
            var handler = OnGhostLocationEvent;
            if (handler != null)
            {
                handler(this, new LocationArgs { RobotId = id, Location = loc });
            }
        }

        public event EventHandler<PolarSpeedArgs> OnSpeedConsigneEvent;
        public virtual void OnSpeedConsigneToRobot(int id, float vLineaire, float vAngulaire)
        {
            var handler = OnSpeedConsigneEvent;
            if (handler != null)
            {
                handler(this, new PolarSpeedArgs { RobotId = id, Vx = vLineaire, Vy = 0, Vtheta = vAngulaire});
            }
        }
    }
}
