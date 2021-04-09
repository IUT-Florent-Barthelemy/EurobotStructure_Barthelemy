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

            //Test de keenan :)
            //wayPointLocation = new Location(3.606, 0, 0, 0, 0, 0);

            

            //Initialisation des vitesse et accélérations souhaitées
            accelLineaire = 1; //en m.s-2
            accelAngulaire = 1 * Math.PI * 1.0; //en rad.s-2

            vitesseLineaireMax = 1.5; //en m.s-1               
            vitesseAngulaireMax = 3 * Math.PI * 1.0; //en rad.s-1
        }

        void InitPositionPID()
        {
            PID_Position_Lineaire = new AsservissementPID(10, 5, 0, 0.5, 0.5, 0.5);
            PID_Position_Angulaire = new AsservissementPID(10, 5, 0, Math.PI/2, Math.PI/2, Math.PI/2);
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
                        depassement = false;
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

                    //===== Depassement =======
                    double thetaCible = Math.Atan2(wayPointLocation.Y - ghostLocationRefTerrain.Y, wayPointLocation.X - ghostLocationRefTerrain.X);
                    double ecartCapCibleRobot = thetaCible - Toolbox.ModuloByAngle(thetaCible, ghostLocationRefTerrain.Theta);

                    bool cibleDevant = true;
                    if(Math.Abs(ecartCapCibleRobot)>Math.PI/2)
                        cibleDevant = false;

                    double coeffMajoration = 2;

                    if (cibleDevant)
                    {
                        //La vitesse doit impérativement tendre à être positive !
                        if (ghostLocationRefTerrain.Vlin < 0)
                        {
                            //On freine en reculant pour revenir à une vitesse positive
                            ghostLocationRefTerrain.Vlin += accelLineaire * samplingPeriod;
                        }
                        else
                        {
                            if (DistanceRestante > stopDistance* coeffMajoration)
                            {
                                if (Math.Abs(ghostLocationRefTerrain.Vlin) < vitesseLineaireMax)
                                    ghostLocationRefTerrain.Vlin += accelLineaire * samplingPeriod;
                                else
                                    ;//rien du tout, on est à Vmax
                            }
                            else
                                ghostLocationRefTerrain.Vlin -= accelLineaire * samplingPeriod;
                        }
                    }
                    else
                    {
                        //La vitesse doit impérativement tendre à être négative !
                        if (ghostLocationRefTerrain.Vlin > 0)
                        {
                            //On freine en avancant pour revenir à une vitesse négative
                            ghostLocationRefTerrain.Vlin -= accelLineaire * samplingPeriod;
                        }
                        else
                        {
                            if (DistanceRestante > stopDistance * coeffMajoration)
                            {
                                if (Math.Abs(ghostLocationRefTerrain.Vlin) < vitesseLineaireMax)
                                    ghostLocationRefTerrain.Vlin -= accelLineaire * samplingPeriod;
                                else
                                    ;//rien du tout, on est à Vmax
                            }
                            else
                                ghostLocationRefTerrain.Vlin += accelLineaire * samplingPeriod;
                        }
                    }


                    if (DistanceRestante < 0.001)
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
            //Calcul de l'erreur angulaire
            double erreurTheta = ghostLocationRefTerrain.Theta - currentLocationRefTerrain.Theta;
            double vAngulaireRobot = PID_Position_Angulaire.CalculatePDoutput(erreurTheta, samplingPeriod);

            //calcul de l'erreur linéaire ghostLocationRefTerrain currentLocationRefTerrain
            PointD GhostPosition = new PointD(ghostLocationRefTerrain.X, ghostLocationRefTerrain.Y);
            PointD ptSegRobot2 = new PointD(currentLocationRefTerrain.X + Math.Cos(currentLocationRefTerrain.Theta), currentLocationRefTerrain.Y + Math.Sin(currentLocationRefTerrain.Theta));
            PointD ptSegRobot1 = new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y);
            PointD ProjGhost = Toolbox.ProjectedPointOnLine(GhostPosition, ptSegRobot1, ptSegRobot2);

            double thetaCible = Math.Atan2(ghostLocationRefTerrain.Y - currentLocationRefTerrain.Y, ghostLocationRefTerrain.X - currentLocationRefTerrain.X);
            double ecartCapCibleRobot = thetaCible - Toolbox.ModuloByAngle(thetaCible, currentLocationRefTerrain.Theta);


            int param = 1;
            if (Math.Abs(ecartCapCibleRobot) > Math.PI / 2)
                param = -1;

            double ErreurLin = Toolbox.Distance(ProjGhost, new PointD(currentLocationRefTerrain.X, currentLocationRefTerrain.Y)) * param;
            Console.WriteLine("ErreurLin: {0}", ErreurLin);
           

            double vLineaireRobot = PID_Position_Lineaire.CalculatePDoutput(ErreurLin, samplingPeriod);

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
