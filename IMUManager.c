using System.Threading;
using Microsoft.SPOT.Hardware;
using SecretLabs.NETMF.Hardware;
using SecretLabs.NETMF.Hardware.Netduino;

namespace NetQuadCopter
{
    public class IMUManager
    {
        /* Declaración de parámetros */
        // PINS
        private const Cpu.Pin AccXPin   = Pins.GPIO_PIN_A4;
        private const Cpu.Pin AccYPin   = Pins.GPIO_PIN_A3;
        private const Cpu.Pin AccZPin   = Pins.GPIO_PIN_A2;
        private const Cpu.Pin GyroXPin  = Pins.GPIO_PIN_A0;
        private const Cpu.Pin GyroYPin  = Pins.GPIO_PIN_A1;

        // Puertos
        private AnalogInput AccXPort, AccYPort, AccZPort, GyroXPort, GyroYPort;

        // Voltages
        private const short Vref = 3300; //mV
        private const short VzeroXG = 1658; //mV (a ojo tras probar) Según el DataSheet son 1500
        private const short VzeroYG = 1612; //mV (a ojo tras probar) Según el DataSheet son 1500
        private const short VzeroZG = 1629; //mV (a ojo tras probar) Según el DataSheet son 1500
        private const short AccSensibility = 300; // mV/g

        private const short VXzeroRate = 1340; //mV (a ojo tras probar) Según el DataSheet son 1350
        private const short VYzeroRate = 1334; //mV (a ojo tras probar)  Según el DataSheet son 1350
        private const byte GyroSensibility = 2; // mV/deg/s (para las salidas normales)
        //private const float GyroSensibility = 9.1F; // mV/deg/s (para las salidas de 4.5X)
        
        private const byte wGyro = 5; // Entre 5 y 20
        private byte T; // Milisegundos entre mediciones de la IMU

        //private const double PIMedios = System.Math.PI / 2;

        /* Declaración de propiedades públicas */
        public float Roll = 0;  // Alabeo
        public float Pitch = 0; // Cabeceo o profundidad
        public float Yaw = 0;   // Dirección

        public float RollInAngles{
            get { return (float)ToAngles(Roll); }
        }
        public float PitchInAngles{
            get { return (float)ToAngles(Pitch); }
        }
        public float YawInAngles{
            get { return (float)ToAngles(Yaw); }
        }


        /* miembros privados */
        private float Rx = -1;
        private float Ry = -1;
        private float Rz = -1;

        private Timer loopTimer;

        /* Variables declaradas globalmente para evitar estar declarandolas continuamente*/
        private int AdcRx;
        private int AdcRy;
        private int AdcRz;
        private int AdcGyroXZ;
        private int AdcGyroYZ;

        private float RxAcc;
        private float RyAcc;
        private float RzAcc;

        private float RxGyro;
        private float RyGyro;

        private int AxzDegress;
        private int AyzDegress;

        private float RateAxz;
        private float RateAyz;

        private float abs;

        // Y estas para debug
        //private long lastCrono;

        /// <summary>
        /// Constructor
        /// </summary>
        /// <param name="interval">Milisegundos entre mediciones del Gyro</param>
        public IMUManager(byte interval)
        {
            T = interval;

            // Inicializo los puertos
            AccXPort = new AnalogInput(AccXPin);
            AccYPort = new AnalogInput(AccYPin);
            AccZPort = new AnalogInput(AccZPin);
            GyroXPort = new AnalogInput(GyroXPin);
            GyroYPort = new AnalogInput(GyroYPin);

            // Inicializo el timer
            loopTimer = new Timer(new System.Threading.TimerCallback(loopTimerCallback), null, 0, T);
        }

        /// <summary>
        /// Devuelve una traza con valores de variables privadas. Usado para depurar
        /// </summary>
        /// <returns></returns>
        public string GetDebug()
        {
            //return "AdcRx: " + AdcRx + ", AdcRy: " + AdcRy + ", AdcRz: " + AdcRz + ", RxAcc: " + RxAcc + ", RyAcc: " + RyAcc + ", RzAcc: " + RzAcc;
            //return "AdcRx: " + AdcRx + ", AdcRy: " + AdcRy + ", AdcRz: " + AdcRz + ", Rx: " + Rx + ", Ry: " + Ry + ", Rz: " + Rz;
            //return "Vx: " + ((AdcRx * Vref / 1023 - 300) / 1000) + ", Vy: " + ((AdcRy * Vref / 1023 - 300) / 1000) + ", Vz: " + ((AdcRz * Vref / 1023 - 300) / 1000);
            //return "AdcGyroXZ: " + AdcGyroXZ + ", AdcGyroYZ: " + AdcGyroYZ + ", RateAxz: " + RateAxz + ", RateAyz: " + RateAyz;
            //return "Pitch: " + PitchInAngles + ", Roll: " + RollInAngles;// +", Yaw: " + ToAngles(Yaw);
            //return "AdcGyroXZ: " + AdcGyroXZ + ", AdcGyroYZ: " + AdcGyroYZ + ", RateAxz: " + RateAxz + ", RateAyz: " + RateAyz;

            return "RxAcc: " + RxAcc + ", RxGyro: " + RxGyro + ", RyAcc: " + RyAcc + ", RyGyro: " + RyGyro;

            //return (lastCrono / System.TimeSpan.TicksPerMillisecond).ToString() + "ms";            
        }


        /// <summary>
        /// Pasa un ángulo en radianes a grados
        /// </summary>
        /// <param name="radians">Ángulo en radianes</param>
        /// <returns>Ángulo en grados</returns>
        private float ToAngles(float radians)
        {
            return (float)(180 * radians / System.Math.PI);
        }


        /// <summary>
        /// Delegado del timer que se encarga de recalcular la posición del cuatricoptero
        /// </summary>
        /// <param name="obj"></param>
        private void loopTimerCallback(object obj)
        {
            //long crono = Utility.GetMachineTime().Ticks;

            #region Cálculo del vector de posición con el acelerómetro

            // Leemos los nuevos valores del acelerómetro
            AdcRx = AccXPort.Read(); // 586;
            AdcRy = AccYPort.Read(); // 630;
            AdcRz = AccZPort.Read(); // 561;

            // y los pasamos a G's
            RxAcc = (AdcRx * Vref / 1023.0f - VzeroXG) / AccSensibility; // g
            RyAcc = (float)(-1.0 * (AdcRy * Vref / 1023.0f - VzeroYG) / AccSensibility);
            RzAcc = (AdcRz * Vref / 1023.0f - VzeroZG) / AccSensibility;
            
            // y por último lo normalizo
            //abs = exMath.Sqrt(RxAcc * RxAcc + RyAcc * RyAcc + RzAcc * RzAcc);
            abs = NetQuadCopter.Math.Sqrt(RxAcc * RxAcc + RyAcc * RyAcc + RzAcc * RzAcc);
            RxAcc = RxAcc / abs;
            RyAcc = RyAcc / abs;
            RzAcc = RzAcc / abs;

            #endregion

            
            #region Cálculo del vector de posición con el giroscopio

            if (Rx == -1 && Ry == -1 && Rz == -1)
            {
                // Es la primera lectura así que no tengo sistema de
                // referencia -> copio los valores del acelerómeetro
                Rx = RxAcc;
                Ry = RyAcc;
                Rz = RzAcc;
            }
            else
            {
                // Leemos los nuevos valores del giroscópio
                AdcGyroXZ = GyroXPort.Read(); // 571;
                AdcGyroYZ = GyroYPort.Read(); // 323;

                // y los pasamos a grados por segundo
                RateAxz = -1 * (AdcGyroXZ * Vref / 1023.0f - VXzeroRate) / GyroSensibility; // deg/s
                RateAyz = (AdcGyroYZ * Vref / 1023.0f - VYzeroRate) / GyroSensibility;

                // Calculamos los valores del vector del Gyro a partir del estado anterior
                AxzDegress = (int)(RateAxz * T / 1000.0f);
                AyzDegress = (int)(RateAyz * T / 1000.0f);
                
                RxGyro = Rx + Microsoft.SPOT.Math.Sin(AxzDegress);
                RyGyro = Ry + Microsoft.SPOT.Math.Sin(AyzDegress);

                // Finalmente calculamos los valores
                Rx = (RxAcc + RxGyro * wGyro) / (1 + wGyro);
                Ry = (RyAcc + RyGyro * wGyro) / (1 + wGyro);
                Rz = RzAcc;// (RzAcc + RzGyro * wGyro) / (1 + wGyro);
            }

            #endregion

            
            #region Finalmente calculamos los valores finales combinando los dos vectores (Acc y Gyro)

            // y normalizamos
            //abs = exMath.Sqrt(Rx*Rx + Ry*Ry + Rz*Rz);
            abs = NetQuadCopter.Math.Sqrt(Rx * Rx + Ry * Ry + Rz * Rz);
            Rx = Rx / abs;
            Ry = Ry / abs;
            Rz = Rz / abs;
            #endregion
            
            //Actualizar Ángulos de Vuelo actuales
            Pitch = NetQuadCopter.Math.Atan2(Rz, Rx);   //  en Radianes
            Roll = NetQuadCopter.Math.Atan2(Ry, Rz) - NetQuadCopter.Math.pio2;    //  en Radianes
            //Yaw = 0; El Yaw lo mantengo siempre en 0 ya que creo que con los sensores que tengo no puedo calcularlo

            // Apunto lo que he tardado en calcular una iteracción del bucle
            //lastCrono = Utility.GetMachineTime().Ticks - crono;
        }
    }
}
