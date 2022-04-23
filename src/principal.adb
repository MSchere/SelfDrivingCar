with Ada.Text_Io, Ada.Real_Time, Sensor, Calefactor, PID;
use Ada.Text_Io, Ada.Real_Time, Sensor, Calefactor;
procedure Principal is
   package Temperaturas_Es is new Float_Io (Temperaturas);
   use Temperaturas_Es;
   subtype Real is Float;
   package Control_Horno is new PID (   Real => Real,
                                     Entrada => Temperaturas,
                                      Salida => Potencias);
   use Control_Horno;

   Controlador_del_Horno: Controlador;
   -- Periodo de muestreo en segundos
   Ts: constant := 1.0;
   Temp, Temp_Ref: Temperaturas;
   Temp_Ambiente: constant Temperaturas := 20.0;
   P: Potencias;
   -- Parámetros de la planta
   Ct: constant := 1_500.0; -- Capacidad térmica [J/ºC]
   Cp: constant := 15.0; -- Coeficiente de pérdidas [W/ºC]
   L: constant := 1.5; -- Retardo [s]
   -- Parámetros de controlador
   Kp, Ki, Kd: Real;
begin
   Put ("Temperatura de referencia: ");
   Get (Temp_Ref);
   -- Calcular Kp, Ki y Kd
   --           -Ls          -Ls 
   --        K.e       1/Cp.e
   -- G(s) = ------- = ---------
   --        1+T.s     1+Ct/Cp.s
   --
   --
   -- Según la regla de Ziegler y Nichols
   -- Kp = 1,2/(R.L)
   -- Ki=Kp/Ti/TsTs y Ti=2.L  
   -- Kd=Kp.Td/Ts y Td=0,5.L 
   declare
      R: Float := Float(Temp_Ref-Temp_Ambiente)/(Ct/Cp);
   begin
      Kp := 1.2/(R*L);
      Ki := Kp/(2.0*L)*Ts;
      Kd := Kp*(0.5*L)/Ts;
   end;
   Programar (Controlador_del_Horno, Kp, Ki, Kd);

   -- Bloque de control
   declare
      Periodo: constant Time_Span := To_Time_Span (Ts);
      Siguiente_Instante: Time := Clock;
      Minutos: constant := Natural (60.0/Ts);
   begin
      for I in 1..10*Minutos loop
         delay until Siguiente_Instante;
         Leer (Temp);
         Put (Temp, Fore=>5, Aft=>2, Exp=>0); New_Line;
         Controlar (Controlador_del_Horno, Temp_Ref, Temp, P);
         Escribir (P);
         Siguiente_Instante := Siguiente_Instante + Periodo;
      end loop;
      Escribir (0.0); -- Para apagar el horno
   end;
end Principal;