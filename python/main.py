import serial
import uinput
import time
import struct

# Ajuste a porta serial e a taxa de baud para corresponder ao seu módulo HC-06
ser = serial.Serial('/dev/rfcomm0', 9600)  # Use '/dev/rfcomm0' para serial Bluetooth

# Cria um dispositivo uinput para teclado e entradas do joystick
device = uinput.Device([
    uinput.KEY_Z,
    uinput.KEY_C,
    uinput.KEY_SPACE,
    uinput.KEY_W,
    uinput.KEY_A,
    uinput.KEY_S,
    uinput.KEY_D,
])

prev_buttons_state = 0  # Estado inicial dos botões

def parse_packet(packet):
    # Desempacota os dados do pacote
    # '>bbB' significa:
    #   >     : Big-endian
    #   b     : Signed char (int8_t) para x1_axis
    #   b     : Signed char (int8_t) para y1_axis
    #   B     : Unsigned char (uint8_t) para buttons
    x1_axis, y1_axis, buttons = struct.unpack('>bbB', packet)
    return x1_axis, y1_axis, buttons

def handle_joystick1(x, y):
    # Mapeia o joystick 1 para as teclas WASD
    # Threshold para determinar se o joystick foi movido significativamente
    threshold = 20  # Ajuste conforme necessário

    # Flags de movimento
    forward = False
    backward = False
    left = False
    right = False

    if y < -threshold:
        # Joystick empurrado para frente
        forward = True
    elif y > threshold:
        # Joystick puxado para trás
        backward = True

    if x < -threshold:
        # Joystick movido para a esquerda
        left = True
    elif x > threshold:
        # Joystick movido para a direita
        right = True

    # Emite eventos de tecla
    # Para frente (W)
    device.emit(uinput.KEY_W, int(forward))
    # Para trás (S)
    device.emit(uinput.KEY_S, int(backward))
    # Esquerda (A)
    device.emit(uinput.KEY_A, int(left))
    # Direita (D)
    device.emit(uinput.KEY_D, int(right))

def handle_buttons(buttons_state):
    global prev_buttons_state

    # Botões representados como bits em buttons_state
    # Bit 0: Botão 0 (LED control, ignorado)
    # Bit 1: Botão 1 ('z' key)
    # Bit 2: Botão 2 ('c' key)
    # Bit 3: Botão 3 (Spacebar key)

    # Mapeamento de bits para códigos de tecla
    button_key_map = {
        1: uinput.KEY_Z,
        2: uinput.KEY_C,
        3: uinput.KEY_SPACE,
    }

    # Para cada botão, verifica se houve mudança de estado
    for btn_index in [1, 2, 3]:
        mask = 1 << btn_index
        is_pressed = (buttons_state & mask) != 0
        was_pressed = (prev_buttons_state & mask) != 0

        if is_pressed and not was_pressed:
            # Botão foi pressionado
            print(f"Button {btn_index} pressed")
            device.emit(button_key_map[btn_index], 1)  # Pressiona a tecla
            device.emit(button_key_map[btn_index], 0)  # Solta a tecla (simula clique)
        elif not is_pressed and was_pressed:
            # Botão foi solto (nenhuma ação necessária para clique único)
            pass

    # Atualiza o estado anterior dos botões
    prev_buttons_state = buttons_state

def main():
    global prev_buttons_state
    try:
        while True:
            # Aguarda o byte de cabeçalho (0xAA)
            header = ser.read(1)
            if header == b'\xAA':
                # Lê os próximos 3 bytes (dados)
                data_bytes = ser.read(3)
                # Lê o byte de fim de pacote (0xFF)
                eop = ser.read(1)
                if eop == b'\xFF':
                    # Desempacota o pacote
                    x1_axis, y1_axis, buttons = parse_packet(data_bytes)

                    # Debug print
                    print(f"X1: {x1_axis}, Y1: {y1_axis}, Buttons: {buttons}")

                    # Trata o joystick 1 (movimento)
                    handle_joystick1(x1_axis, y1_axis)

                    # Trata os botões
                    handle_buttons(buttons)

                else:
                    # Byte de fim de pacote inválido, descarta e continua
                    continue
            else:
                # Descarta bytes até encontrar o byte de cabeçalho
                continue

    except KeyboardInterrupt:
        print("Programa interrompido pelo usuário")
    except Exception as e:
        print(f"Ocorreu um erro: {e}")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
