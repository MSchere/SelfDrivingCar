with Ada.Text_io; use Ada.Text_io;
package body PIDvel is --Este controlador funciona de manera muy similar al del calefactor
   procedure Programar (el_Controlador: in out Controlador; Kp, Ki, Kd: Real) is
   begin
      el_Controlador := (Kp=> Kp, Ki=> Ki, Kd=> Kd, S_Anterior=> 0.0, Error_Anterior=> 0.0);
      put_line("Programado con valores: ");
      put_line("Kp: " & Real'Image(el_Controlador.Kp));
      put_line("Ki: " & Real'Image(el_Controlador.Ki));
      put_line("Kd: " & Real'Image(el_Controlador.Kd));
   end Programar;

   procedure Controlar (con_el_Controlador: in out Controlador;
                                      R, C:     Entrada;
                                         U: out Salida) is
      Cont: Controlador renames con_el_Controlador;
      S, Error_Actual, Up, Ui, Ud, Res, Min, Max: Real;
   begin
        min := 0.00001;
        max := 500.0;
      -- Cálculos de control
      put_line("---Controlador velocidad---");
       Error_Actual := Real(R-C);
       put_line("Error acumulado: " & Real'Image(con_el_Controlador.S_Anterior));
       --put_line("Error anterior: " & Real'Image(con_el_Controlador.Error_Anterior));
       put_line("Error actual: " & Real'Image(Error_Actual));
         -- kp.e(n)
      Up := Cont.Kp * Error_Actual;
         -- s(n) = s(n-1) + e(n)
       S := Real(Cont.S_Anterior) + Error_Actual;
         -- ki.s(n)
      Ui := Real(Cont.Ki) * S;
         -- kd.[e(n)-e(n-1)]
      Ud := Real(Cont.Kd) * (Error_Actual - Real(Cont.Error_Anterior));

      Res := Up + Ui + Ud;
       U := Salida(Res);
       --Antiwindup
      if (res < min) then
        --put_line("Velocidad mínima");
        U:= Salida(min);
      elsif (res > max) then
        --put_line("Velocidad máxima");
        U:= Salida(max);
      else
        --put_line("Control Automático");
        con_el_Controlador.S_Anterior := con_el_Controlador.S_Anterior + con_el_Controlador.Error_Anterior;
        U:= Salida(res);
      end if;
      -- Actualización del estado
      Cont.Error_Anterior := Error_Actual;
   end Controlar;
end PIDvel;
