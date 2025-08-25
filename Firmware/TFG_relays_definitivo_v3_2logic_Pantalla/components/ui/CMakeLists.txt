
idf_component_register(
    SRCS "ui.c" "ui_helpers.c" "ui_events.h" 
        	"screens/ui_Menu_Inicio.c"
"screens/ui_AMBIENTES1.c"
"screens/ui_SUELO.c"
"screens/ui_SECTORES.c"
"screens/ui_AMBIENTES2.c"
"screens/ui_Nebulizar.c"
"screens/ui_CONFIGURACION.c"
"screens/ui_GraficasAmbiente.c"
"screens/ui_GraficaTempHumS1.c"
"screens/ui_ScreenINFODPV.c"
"screens/ui_GraficaTempHumS2.c"
"screens/ui_RANGOSSectoresDPV.c"
"screens/ui_RANGOSDPVS1MAX.c"
"screens/ui_RANGOSDPVS2MAX.c"
"screens/ui_RANGOSDPVS1MIN.c"
"screens/ui_RANGOSDPVS2MIN.c"
"screens/ui_BalanceDiarioS1.c"
"screens/ui_BalanceDiarioS2.c"


         "components/ui_comp_hook.c"
    INCLUDE_DIRS "."
    REQUIRES lvgl esp_lcd driver freertos log  esp_lcd_touch
)