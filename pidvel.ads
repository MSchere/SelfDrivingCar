generic
   type Real    is digits <>;
   type Entrada is digits <>;
   type Salida  is digits <>;
package PIDvel is
   type Controlador is limited private;

   procedure Programar (el_Controlador: in out Controlador; Kp, Ki, Kd: Real);
   procedure Controlar(con_el_Controlador: in out Controlador;
                                     R, C:        Entrada;
                                        U: out    Salida);
private
   type Controlador is record
      -- Par�metros del controlador
      Kp, Ki, Kd: Real;
      -- Estado del controlador
      S_Anterior    : Real := 0.0; -- s(n-1) Condiciones de
      Error_Anterior: Real := 0.0; -- e(n-1) reposo inicial
   end record;
end PIDvel;
