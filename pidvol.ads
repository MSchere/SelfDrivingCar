generic
   type Real    is digits <>;
   type Entrada is digits <>;
   type Salida  is digits <>;
package PIDvol is
   type Controlador is limited private;

   procedure Programar (el_Controlador: in out Controlador; Kp: Real);
   procedure Controlar(con_el_Controlador: in out Controlador;
                                     Ref, Err_lat, Err_ang:        Entrada;
                                        U: out    Salida);
private
   type Controlador is record
      -- Par�metros del controlador
      Kp: Real;
   end record;
end PIDvol;
