with Ada.Text_io; use Ada.Text_io;
package body PID is
   procedure Programar (el_Controlador: in out Controlador; Kp, Ki, Kd: Real; Modo: Boolean) is
   begin
      el_Controlador := (Kp=> Kp, Ki=> Ki, Kd=> Kd, S_Anterior=> 0.0, Error_Anterior=> 0.0, Modo=> Modo);
      put_line("Programado con valores: ");
      put_line("Kp: " & Real'Image(el_Controlador.Kp));
      put_line("Ki: " & Real'Image(el_Controlador.Ki));
      put_line("Kd: " & Real'Image(el_Controlador.Kd));
      put_line("Modo: " & Integer'Image(el_Controlador.Modo));
   end Programar;

   procedure Controlar (con_el_Controlador: in out Controlador;
                                      R, C:     Entrada;
                                         U: out Salida) is
      Cont: Controlador renames con_el_Controlador;
      Error_Actual: Real;
      S: Real;
      Up, Ui, Ud, Res, Min, Max: Real;
   begin
      -- C�lculos de control
       Error_Actual := Real(R-C);
       put_line("");
       put_line("Error acumulado: " & Real'Image(con_el_Controlador.S_Anterior));
       put_line("Error anterior: " & Real'Image(con_el_Controlador.Error_Anterior));
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
       if (modo) then
         Min := 0.00001;
         Max := 500.0;
       else
         Min := -1.0;
         Max := 1.0;
       end if;

      if (res < min) then
        put_line("Potencia negativa");
        U:= Salida(min);
        --con_el_Controlador.S_Anterior := Ti/Ts*(res/Kp-Error_Actual-Td/Ts*(Error_Actual - con_el_Controlador.Error_Anterior));
      elsif (res > max) then
        put_line("Demasiada potencia");
        U:= Salida(max);
        --con_el_Controlador.S_Anterior := Ti/Ts*(res/Kp-Error_Actual-Td/Ts*(Error_Actual - con_el_Controlador.Error_Anterior));
      else
        put_line("Control Automático");
        con_el_Controlador.S_Anterior := con_el_Controlador.S_Anterior + con_el_Controlador.Error_Anterior;
        U:= Salida(res);
      end if;
      -- Actualizaci�n del estado
      Cont.Error_Anterior := Error_Actual;
   end Controlar;
end PID;
