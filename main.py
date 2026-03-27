
# ============== Configuración motores y sensores
mA = nezhaV2.MotorPostion.M1
mB = nezhaV2.MotorPostion.M2
mC = nezhaV2.MotorPostion.M3
mD = nezhaV2.MotorPostion.M4

# ============== Funciones y clases

class Robot():
    import Math
    

    def __init__(self, wheel_diameter=100, wheel_distance=100, mi=nezhaV2.MotorPostion.M1, md=nezhaV2.MotorPostion.M2):
        self.mi = mi
        self.md = md

        self.wheel_diameter = wheel_diameter
        self.wheel_distance = wheel_distance
        self.perimeter = (self.wheel_diameter)*Math.PI

        nezhaV2.set_wheel_perimeter(self.perimeter, nezhaV2.Uint.CM)
        nezhaV2.set_combo_motor(mi, md)


    def mover(self, distance, max_speed):
        import Math

        grados = (distance / self.perimeter) * 360

        if max_speed > 100:
            max_speed = 100
        elif max_speed < -100:
            max_speed = -100

        # Velocidad mínima para que los motores arranquen
        min_speed = 15
        # Porcentaje de la distancia dedicado a acelerar y frenar (cada uno)
        ramp_ratio = 0.3

        accel_grados = abs(grados) * ramp_ratio
        decel_grados = abs(grados) * ramp_ratio

        direction = 1 if distance > 0 else -1

        if distance != 0:
            nezhaV2.reset_rel_angle_value(self.mi)
            nezhaV2.start(self.mi, min_speed * -direction)
            nezhaV2.start(self.md, min_speed * direction)

            while True:
                current = abs(nezhaV2.read_rel_angle(self.mi))
                remaining = abs(grados) - current

                if current <= accel_grados:
                    # Fase de aceleración
                    t = current / accel_grados  # 0 → 1
                    speed = min_speed + (max_speed - min_speed) * t

                elif remaining <= decel_grados:
                    # Fase de deceleración
                    t = remaining / decel_grados  # 1 → 0
                    speed = min_speed + (max_speed - min_speed) * t

                else:
                    # Velocidad crucero
                    speed = max_speed

                speed = max(min_speed, min(max_speed, speed))

                nezhaV2.start(self.mi, speed * -direction)
                nezhaV2.start(self.md, speed * direction)

                if current >= abs(grados):
                    break

            nezhaV2.stop(self.mi)
            nezhaV2.stop(self.md)

        else:
            nezhaV2.combo_stop()

    def girar(self, angle, speed=50, mode="both"):
        """
        Gira el robot un ángulo dado usando los encoders.
        - angle: grados a girar (positivo = derecha, negativo = izquierda)
        - speed: velocidad (0-100)
        - mode: "both"  → ambas ruedas en sentido contrario (giro en punto)
                "left"  → solo gira la rueda izquierda
                "right" → solo gira la rueda derecha
        """

        if speed > 100:
            speed = 100
        elif speed < 0:
            speed = 0

        direction = 1 if angle > 0 else -1

        # Calcula cuántos grados debe girar cada motor según el modo
        # El arco que debe recorrer la rueda exterior depende de la distancia entre ruedas
        if mode == "both":
            # Ambas ruedas recorren la mitad del arco total cada una
            arco = (abs(angle) / 360) * Math.PI * self.wheel_distance
            grados_motor = (arco / self.perimeter) * 360

        elif mode == "left" or mode == "right":
            # Una sola rueda recorre el arco completo alrededor de la rueda parada
            arco = (abs(angle) / 360) * Math.PI * (self.wheel_distance * 2)
            grados_motor = (arco / self.perimeter) * 360

        else:
            return  # modo no válido

        nezhaV2.reset_rel_angle_value(self.mi)
        nezhaV2.reset_rel_angle_value(self.md)

        if mode == "both":
            nezhaV2.start(self.mi,  speed * -direction)
            nezhaV2.start(self.md, -speed * direction)

            while True:
                if abs(nezhaV2.read_rel_angle(self.mi)) >= grados_motor:
                    break

        elif mode == "right":
            # Giro a la derecha con rueda derecha → rueda izq parada
            # Giro a la izquierda con rueda derecha → rueda der mueve hacia atrás
            nezhaV2.start(self.mi, 0)
            nezhaV2.start(self.md, -speed * direction)

            while True:
                if abs(nezhaV2.read_rel_angle(self.md)) >= grados_motor:
                    break

        elif mode == "left":
            # Giro a la derecha con rueda izquierda → rueda izq mueve hacia adelante
            nezhaV2.start(self.mi, speed * -direction)
            nezhaV2.start(self.md, 0)

            while True:
                if abs(nezhaV2.read_rel_angle(self.mi)) >= grados_motor:
                    break

        nezhaV2.stop(self.mi)
        nezhaV2.stop(self.md)
    def follow_line(self, base_speed=50, stop_on_lost=False):
        """
        Sigue una línea usando el sensor Trackbit.
        - base_speed: velocidad base (0-100)
        - stop_on_lost: si True, para al perder la línea completamente
        """
        PlanetX_Basic.Trackbit_get_state_value()

        # ── Recto: línea centrada ──────────────────────────────
        if PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_1):
            nezhaV2.start(self.mi, -base_speed)
            nezhaV2.start(self.md, base_speed)

        # ── Desvío suave a la izquierda ────────────────────────
        elif (PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_3) or
            PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_11)):
            nezhaV2.start(self.mi, -int(base_speed * 0.1))   # motor izq lento
            nezhaV2.start(self.md, base_speed)               # motor der rápido

        # ── Desvío fuerte a la izquierda ───────────────────────
        elif PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_8):
            nezhaV2.start(self.mi, 0)
            nezhaV2.start(self.md, base_speed)

        # ── Desvío suave a la derecha ──────────────────────────
        elif (PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_2) or
            PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_14)):
            nezhaV2.start(self.mi, -base_speed)
            nezhaV2.start(self.md, int(base_speed * 0.1))   # motor der lento

        # ── Desvío fuerte a la derecha ─────────────────────────
        elif PlanetX_Basic.trackbit_state(PlanetX_Basic.TrackbitStateType.TRACKING_STATE_12):
            nezhaV2.start(self.mi, -base_speed)
            nezhaV2.start(self.md, 0)

        # ── Línea perdida ──────────────────────────────────────
        else:
            if stop_on_lost:
                nezhaV2.stop(self.mi)
                nezhaV2.stop(self.md)
            # Si stop_on_lost=False simplemente mantiene el último movimiento


    def follow_line_distance(self, distance, base_speed=50):
        """
        Sigue la línea hasta recorrer una distancia en cm.
        """
        grados = (distance / self.perimeter) * 360

        nezhaV2.reset_rel_angle_value(self.mi)

        while abs(nezhaV2.read_rel_angle(self.mi)) < grados:
            self.follow_line(base_speed)

        nezhaV2.stop(self.mi)
        nezhaV2.stop(self.md)

    def follow_line_until_cross(self, base_speed=50, channel=PlanetX_Basic.TrackbitStateType.TRACKING_STATE_1):
        """
        Sigue la línea hasta que el sensor detecte un cruce en el canal indicado.
        Por defecto usa TRACKING_STATE_1 (todos los sensores activos = cruce).
        """
        PlanetX_Basic.Trackbit_get_state_value()

        while not PlanetX_Basic.trackbit_state(channel):
            self.follow_line(base_speed)
            PlanetX_Basic.Trackbit_get_state_value()

        nezhaV2.stop(self.mi)
        nezhaV2.stop(self.md)

# ============== Código

robot = Robot(56, 120, mA, mB)

robot.mover(400,70)
robot.girar(-120, 30, "both")
robot.mover(-400,70)

# robot.mover(100,50)

# robot.follow_line_distance(1000, 60)+

# robot.follow_line_distance(500, base_speed=60)





