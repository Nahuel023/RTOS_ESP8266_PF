
# Robot Control System RTOS_ESP8266_PF

Este proyecto es un sistema de control para un robot diferencial utilizando un servidor HTTP para la interfaz web y un sistema operativo en tiempo real (FREERTOS) para la gestión de tareas. 
El sistema permite controlar el movimiento del robot, activar o desactivar el modo de esquivar objetos, y muestra la distancia medida por un sensor de ultrasonido.

## Funcionalidades

1. **Control del Movimiento**: Permite mover el robot en cuatro direcciones (adelante, atrás, izquierda, derecha) y detenerlo.
2. **Modo de Esquivar Objetos**: Permite activar o desactivar el modo de esquivar objetos. Cuando está activado, el botón correspondiente cambia de color y texto.
3. **Medición de Distancia**: Muestra la distancia medida por el sensor de ultrasonido en tiempo real.

## Estructura del Código

### `main.c`

Este es el archivo principal que maneja las solicitudes HTTP y controla el robot. Las funciones y tareas clave son:

- **Funciones Principales**:
  - `handle_client(int client_socket)`: Maneja las solicitudes HTTP entrantes y responde con la página HTML o ejecuta comandos para controlar el robot.
  - `MotorControl(int8_t setMotorRight, int8_t setMotorLeft)`: Controla la velocidad de los motores derecho e izquierdo del robot.

- **Tareas del RTOS**:
  - `task_HC_SR04(void* ignore)`: Mide la distancia utilizando un sensor de ultrasonido HC-SR04 y actualiza la variable global `distance`.
  - `task_blink(void* ignore)`: Parpadea un LED conectado al GPIO16 para indicar que el sistema está funcionando.
  - `task_AvoidObstacle(void* ignore)`: Activa el modo de esquivar objetos, girando el robot si detecta un obstáculo a una distancia menor a la definida.
  - `task_http_server(void* ignore)`: Maneja el servidor HTTP para controlar el robot a través de una página web.

### Página Web

La página web está integrada en el código C y se sirve a través del servidor HTTP. Incluye botones para controlar el movimiento del robot y un botón para activar/desactivar el modo de esquivar objetos.

#### Botones de Control

- Adelante: `fetch('/forward')`
- Atrás: `fetch('/backward')`
- Izquierda: `fetch('/left')`
- Derecha: `fetch('/right')`

#### Botón de Esquivar Objetos

- `fetch('/avoid')`: Activa o desactiva el modo de esquivar objetos y cambia el color y texto del botón.

## Configuración y Ejecución

  1. Clona este repositorio en tu máquina local:
   ```bash
   git clone https://github.com/Nahuel023/RTOS_ESP8266_PF.git
   ```
  2. Compila el proyecto utilizando tu entorno de desarrollo preferido (por ejemplo, PlatformIO).
  3. Sube el firmware al microcontrolador.
  4. Conecta al servidor HTTP del robot utilizando un navegador web.

## Ejemplo de Uso
1. **Abrir la Página Web:** Accede a la dirección IP del robot desde un navegador web.
2. **Control del Movimiento:** Usa los botones para mover el robot en la dirección deseada.
3. **Activar/Desactivar Esquivar Objetos:** Haz clic en el botón "Esquivar Objetos ON/OFF" para activar o desactivar el modo de esquivar objetos. El botón cambiará de color y texto para reflejar el estado actual.
4. **Ver la Distancia:** La distancia medida por el sensor de ultrasonido se actualizará en tiempo real en la página web.

## Contribuciones
Las contribuciones son bienvenidas. Si tienes sugerencias, errores que informar o mejoras que proponer, por favor abre un issue o envía un pull request.


## Autores

- [@Nahuel023](https://www.github.com/Nahuel023)


## License

[MIT](https://choosealicense.com/licenses/mit/)

