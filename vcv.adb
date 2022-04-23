with Ada.Text_IO;             use Ada.Text_IO;
with GNAT.Sockets;            use GNAT.Sockets;
with Ada.Calendar;		use Ada.Calendar;
with Ada.Text_Io; use Ada.Text_Io;
with Ada.Streams;
with Ada.Exceptions;
with Ada.Strings.Unbounded;
with Ada.Numerics.Elementary_Functions;
with Ada.Float_Text_Io; use Ada.Float_Text_Io;
with PIDvel; --Mis controladores
with PIDvol;

use type Ada.Streams.Stream_Element_Count;
use Ada.Numerics.Elementary_Functions;

with Ada.Text_Io, Ada.Real_Time;

procedure Vcv is
  package PID_Vel is new PIDvel(Float, Float, Float); --Inicializamos el paquete
  package PID_Vol is new PIDvol(Float, Float, Float);
  Cont_Vel: PID_Vel.Controlador; --Instanciamos el controlador de velocidad
  Cont_Vol: PID_Vol.Controlador; --Instanciamos el controlador de volante
  package Ent_ES is new Integer_IO(Integer);
  Client : Socket_Type;
  Address: Sock_Addr_Type;
  Channel: Stream_Access;
  Send: String := (1 => ASCII.CR, 2 => ASCII.LF, 3 => ASCII.CR, 4 => ASCII.LF);
  Offset: Ada.Streams.Stream_Element_Count;
  Data: Ada.Streams.Stream_Element_Array (1 .. 1);
  RetMsg: Ada.Strings.Unbounded.Unbounded_String := Ada.Strings.Unbounded.Null_Unbounded_String;
  ControlMsg: Ada.Strings.Unbounded.Unbounded_String := Ada.Strings.Unbounded.Null_Unbounded_String;
  type err_ang_t is digits 3 range -3.15 .. 3.15;
  type err_lat_t is digits 4 range -99.99 .. 99.99;
  type velocidad_t is digits 5 range 0.00 .. 500.00;
  type distancia_t is digits 6 range -9999.00 .. 9999.0;
  type volante_t is digits 3 range -1.0 .. 1.0;
  type accFreno_t is digits 3 range -1.0 .. 1.0;
  type radio_t is digits 7 range 0.0 .. 99999.99;
  c: Character;
  j: Integer := 1;
  count: Integer := 0;
	Kp, Ki, Kd: Float;
  Err_ang: err_ang_t;
  Err_lat: err_lat_t;
  Curvatura, CurvaturaSig, Tipo, TipoSig: Float;
  dist, distSig: distancia_t;
  velocidad, vel_max, vel_min, ref_Velocidad, vel_calculada: velocidad_t;
  ref_Volante, volante: volante_t := 0.0;
  --El coche puede completar varios circuitos con curvas pronunciadas a excepción
  --de los que por alguna razón TORCS no detecta la curva.

  --La velocidad mínima debe de ser una velocidad tal que el coche pueda
  --superar la curva más agresiva del circuito sin salirse de la pista.
  --La velocidad máxima por lo general no debe de superar los 200 Km/h para no
  --tener demasiado error acumulado. (150 Km/h suele ir bien para circuitos sin
  --curvas demasiado pronunciadas). Para los circuitos ovalados en los que no es
  --necesario frenar simplemente presionar enter al principio del programa
  --para introducir los valores máximos de velocidad máxima y mínima.
  procedure introduce_vel is
  begin
    Put_Line("Parámetros de velocidad (presionar enter para velocidad máxima)");
      Put("Introduce la velocidad mínima en Km/h: ");
      vel_min := velocidad_t(Float'Value(get_line)/3.6);
      Put("Introduce la velocidad máxima en Km/h: ");
      vel_max := velocidad_t(Float'Value(get_line)/3.6);
      exception
        when ocurrencia : Constraint_Error => put(Ada.Exceptions.Exception_Information(ocurrencia));
        put_line("El valor introducido es incorrecto, introduciendo valores máximos");
        vel_min := velocidad_t(500.00);
        vel_max := velocidad_t(500.00);
    end introduce_vel;
begin
  introduce_vel;
  GNAT.Sockets.Initialize;  -- initialize a specific package
  Create_Socket (Client);
  Address.Addr := Inet_Addr("127.0.0.1");
  Address.Port := 12321;
  Connect_Socket (Client, Address);
  Channel := Stream (Client);
  Kp := 0.02;
  Ki := 0.02;
  Kd := 0.01;

  PID_Vel.Programar(Cont_Vel, Kp, Ki, Kd);--Programamamos el controlador de velocidad
  PID_Vol.Programar(Cont_Vol, Kp);--Programamamos el controlador de volante

   -- Esta es la cadena de configuración. Cambiar los números para los distintos modos
  String'Write (Channel, "Controller init request 32" & Send); --solo funciona para el modo 32!
  Curvatura := 0.0;CurvaturaSig := 1.0; distSig:=0.0; velocidad := 0.0;
  while Curvatura/=0.0 or CurvaturaSig /= 0.0 or distSig /= 0.0 or velocidad/=0.0  loop
    count := 0;
    ControlMsg := Ada.Strings.Unbounded.Null_Unbounded_String;
    RetMsg := Ada.Strings.Unbounded.Null_Unbounded_String;
    loop
		    Ada.Streams.Read (Channel.All, Data, Offset);
		    Ada.Strings.Unbounded.Append (Source => RetMsg, New_Item => Character'Val (Data(1)));
		 exit when Character'Val (Data(1)) = '*' ;
     end loop;
     ---- Lectura de los datos que manda el coche
     --Ada.Text_IO.Put_Line (Ada.Strings.Unbounded.To_String (RetMsg));
      j:=1;
      for i in Ada.Strings.Unbounded.To_String (RetMsg)'Range loop
	      c := Ada.Strings.Unbounded.To_String (RetMsg)(i);
	      if (c = ';' or c = '*') then
	      count := count + 1;
	 case count is --Está comentado lo que no nos interesa
	   when 1 => Err_ang := err_ang_t'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   when 2 => Err_lat := err_lat_t'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   --when 3 => Tipo := Float'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   --when 4 => Curvatura := Float'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   when 5 => dist := distancia_t'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   --when 6 => TipoSig := Float'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   --when 7 => CurvaturaSig := Float'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   when 8 => distSig := distancia_t'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   when 9 => velocidad := velocidad_t'Value(Ada.Strings.Unbounded.To_String (RetMsg)(j..i-1));
	   when others => NULL;
	 end case;
	    j := i+1;
	      end if;
  end loop;
  -------------------------------Código de control------------------------------
  put_line("");
  if (vel_max = 500.0) then
    put_line("Modo: velocidad máxima");
    --Nunca frena
    ref_Velocidad := vel_max;
  else
    put_line("Modo: velocidad inteligente");
    --Velocidad inteligente basada en la distancia al siguiente segmento
    ref_Velocidad := velocidad_t(Float(vel_min) + Float(vel_max) * 0.002 * Float(abs dist));
  end if;
  PID_Vel.Controlar(Cont_Vel, Float(ref_Velocidad), Float(velocidad), Float(vel_calculada));
  PID_Vol.Controlar(Cont_Vol, Float(ref_Volante), Float(Err_lat), Float(Err_ang), Float(volante));
  put_line("------Datos del coche------");
  put_line("Distancia a segmento: " & distancia_t'image(dist));
  put_line("Velocidad: " & velocidad_t'image(vel_calculada));
  put_line("Volante: " & volante_t'image(volante));
  ------------------------------------------------------------------------------
  Ada.Strings.Unbounded.Append (Source => ControlMsg, New_Item => volante_t'image(volante));
  Ada.Strings.Unbounded.Append (Source => ControlMsg, New_Item => ';');
  Ada.Strings.Unbounded.Append (Source => ControlMsg, New_Item => velocidad_t'image(vel_calculada));
  Ada.Strings.Unbounded.Append (Source => ControlMsg, New_Item => '*');
  String'Write (Channel,  Ada.Strings.Unbounded.To_String (ControlMsg) & Send);
end loop;
----- Comando para terminar la carrera
  String'Write (Channel, "Controller end request 44" & Send);
end Vcv;
