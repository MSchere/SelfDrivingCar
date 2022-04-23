with Ada.Text_io; use Ada.Text_io;
with Ada.Numerics;
package body PIDvol is --Este controlador solo emplea la parte proporcional
   procedure Programar (el_Controlador: in out Controlador; Kp: Real) is
   begin
      el_Controlador := (Kp=> Kp);
      put_line("Programado con valores: ");
      put_line("Kp: " & Real'Image(el_Controlador.Kp));
   end Programar;

   procedure Controlar (con_el_Controlador: in out Controlador;
                    Ref, Err_lat, Err_ang:     Entrada;
                                         U: out Salida) is
      Cont: Controlador renames con_el_Controlador;
      Res, Min, Max, Err, Err_ang_ampl, Err_lat_ampl: Real;
   begin
    Min := Real(-1);
    Max := Real(1);
    -- Cálculos de control
    Err_ang_ampl := Real(Err_ang)*100.0; --Error angular amplificado
    Err_lat_ampl := Real(Err_lat)*10.0; --Error lateral amplificado
    Err := Err_ang_ampl + Err_lat_ampl;
    put_line("----Controlador volante----");
    --put_line("Error acumulado: " & Real'Image(con_el_Controlador.S_Anterior));
    --put_line("Error anterior: " & Real'Image(con_el_Controlador.Error_Anterior));
    put_line("Error lateral: " & Real'Image(Err_lat_ampl));
    put_line("Error angular: " & Real'Image(Err_ang_ampl));
    put_line("Error total: " & Real'Image(Err));

      Res := Cont.Kp * Err;
      U := Salida(Res);

       --Control de límites
      if (res < min) then
        --put_line("Volante en posición mínima");
        U:= Salida(min);
      elsif (res > max) then
        --put_line("Volante en posición máxima");
        U:= Salida(max);
      else
        --put_line("Control Automático");
        U:= Salida(res);
      end if;
   end Controlar;
end PIDvol;
