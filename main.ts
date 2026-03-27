//  ============== Configuración motores y sensores
let mA = nezhaV2.MotorPostion.M1
let mB = nezhaV2.MotorPostion.M2
let mC = nezhaV2.MotorPostion.M3
let mD = nezhaV2.MotorPostion.M4
//  ============== Funciones y clases
class Robot {
    mi: number
    md: number
    wheel_diameter: number
    wheel_distance: number
    perimeter: number
    public static __initRobot() {
    }
    
    constructor(wheel_diameter: number = 100, wheel_distance: number = 100, mi: number = nezhaV2.MotorPostion.M1, md: number = nezhaV2.MotorPostion.M2) {
        this.mi = mi
        this.md = md
        this.wheel_diameter = wheel_diameter
        this.wheel_distance = wheel_distance
        this.perimeter = this.wheel_diameter * Math.PI
        nezhaV2.setWheelPerimeter(this.perimeter, nezhaV2.Uint.cm)
        nezhaV2.setComboMotor(mi, md)
    }
    
    public mover(distance: number, max_speed: number) {
        let current: number;
        let remaining: number;
        let t: number;
        let speed: number;
        let grados = distance / this.perimeter * 360
        if (max_speed > 100) {
            max_speed = 100
        } else if (max_speed < -100) {
            max_speed = -100
        }
        
        //  Velocidad mínima para que los motores arranquen
        let min_speed = 15
        //  Porcentaje de la distancia dedicado a acelerar y frenar (cada uno)
        let ramp_ratio = 0.3
        let accel_grados = Math.abs(grados) * ramp_ratio
        let decel_grados = Math.abs(grados) * ramp_ratio
        let direction = distance > 0 ? 1 : -1
        if (distance != 0) {
            nezhaV2.resetRelAngleValue(this.mi)
            nezhaV2.start(this.mi, min_speed * -direction)
            nezhaV2.start(this.md, min_speed * direction)
            while (true) {
                current = Math.abs(nezhaV2.readRelAngle(this.mi))
                remaining = Math.abs(grados) - current
                if (current <= accel_grados) {
                    //  Fase de aceleración
                    t = current / accel_grados
                    //  0 → 1
                    speed = min_speed + (max_speed - min_speed) * t
                } else if (remaining <= decel_grados) {
                    //  Fase de deceleración
                    t = remaining / decel_grados
                    //  1 → 0
                    speed = min_speed + (max_speed - min_speed) * t
                } else {
                    //  Velocidad crucero
                    speed = max_speed
                }
                
                speed = Math.max(min_speed, Math.min(max_speed, speed))
                nezhaV2.start(this.mi, speed * -direction)
                nezhaV2.start(this.md, speed * direction)
                if (current >= Math.abs(grados)) {
                    break
                }
                
            }
            nezhaV2.stop(this.mi)
            nezhaV2.stop(this.md)
        } else {
            nezhaV2.comboStop()
        }
        
    }
    
    public girar(angle: number, speed: number = 50, mode: string = "both") {
        let arco: number;
        let grados_motor: number;
        /** 
        Gira el robot un ángulo dado usando los encoders.
        - angle: grados a girar (positivo = derecha, negativo = izquierda)
        - speed: velocidad (0-100)
        - mode: "both"  → ambas ruedas en sentido contrario (giro en punto)
                "left"  → solo gira la rueda izquierda
                "right" → solo gira la rueda derecha
        
 */
        if (speed > 100) {
            speed = 100
        } else if (speed < 0) {
            speed = 0
        }
        
        let direction = angle > 0 ? 1 : -1
        //  Calcula cuántos grados debe girar cada motor según el modo
        //  El arco que debe recorrer la rueda exterior depende de la distancia entre ruedas
        if (mode == "both") {
            //  Ambas ruedas recorren la mitad del arco total cada una
            arco = Math.abs(angle) / 360 * Math.PI * this.wheel_distance
            grados_motor = arco / this.perimeter * 360
        } else if (mode == "left" || mode == "right") {
            //  Una sola rueda recorre el arco completo alrededor de la rueda parada
            arco = Math.abs(angle) / 360 * Math.PI * (this.wheel_distance * 2)
            grados_motor = arco / this.perimeter * 360
        } else {
            return
        }
        
        //  modo no válido
        nezhaV2.resetRelAngleValue(this.mi)
        nezhaV2.resetRelAngleValue(this.md)
        if (mode == "both") {
            nezhaV2.start(this.mi, speed * -direction)
            nezhaV2.start(this.md, -speed * direction)
            while (true) {
                if (Math.abs(nezhaV2.readRelAngle(this.mi)) >= grados_motor) {
                    break
                }
                
            }
        } else if (mode == "right") {
            //  Giro a la derecha con rueda derecha → rueda izq parada
            //  Giro a la izquierda con rueda derecha → rueda der mueve hacia atrás
            nezhaV2.start(this.mi, 0)
            nezhaV2.start(this.md, -speed * direction)
            while (true) {
                if (Math.abs(nezhaV2.readRelAngle(this.md)) >= grados_motor) {
                    break
                }
                
            }
        } else if (mode == "left") {
            //  Giro a la derecha con rueda izquierda → rueda izq mueve hacia adelante
            nezhaV2.start(this.mi, speed * -direction)
            nezhaV2.start(this.md, 0)
            while (true) {
                if (Math.abs(nezhaV2.readRelAngle(this.mi)) >= grados_motor) {
                    break
                }
                
            }
        }
        
        nezhaV2.stop(this.mi)
        nezhaV2.stop(this.md)
    }
    
    public follow_line(base_speed: number = 50, stop_on_lost: boolean = false) {
        /** 
        Sigue una línea usando el sensor Trackbit.
        - base_speed: velocidad base (0-100)
        - stop_on_lost: si True, para al perder la línea completamente
        
 */
        PlanetX_Basic.Trackbit_get_state_value()
        //  ── Recto: línea centrada ──────────────────────────────
        if (PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_1)) {
            nezhaV2.start(this.mi, -base_speed)
            nezhaV2.start(this.md, base_speed)
        } else if (PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_3) || PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_11)) {
            //  ── Desvío suave a la izquierda ────────────────────────
            nezhaV2.start(this.mi, -Math.trunc(base_speed * 0.1))
            //  motor izq lento
            nezhaV2.start(this.md, base_speed)
        } else if (PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_8)) {
            //  motor der rápido
            //  ── Desvío fuerte a la izquierda ───────────────────────
            nezhaV2.start(this.mi, 0)
            nezhaV2.start(this.md, base_speed)
        } else if (PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_2) || PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_14)) {
            //  ── Desvío suave a la derecha ──────────────────────────
            nezhaV2.start(this.mi, -base_speed)
            nezhaV2.start(this.md, Math.trunc(base_speed * 0.1))
        } else if (PlanetX_Basic.TrackbitState(PlanetX_Basic.TrackbitStateType.Tracking_State_12)) {
            //  motor der lento
            //  ── Desvío fuerte a la derecha ─────────────────────────
            nezhaV2.start(this.mi, -base_speed)
            nezhaV2.start(this.md, 0)
        } else if (stop_on_lost) {
            nezhaV2.stop(this.mi)
            nezhaV2.stop(this.md)
        }
        
    }
    
    //  Si stop_on_lost=False simplemente mantiene el último movimiento
    public follow_line_distance(distance: number, base_speed: number = 50) {
        /** Sigue la línea hasta recorrer una distancia en cm. */
        let grados = distance / this.perimeter * 360
        nezhaV2.resetRelAngleValue(this.mi)
        while (Math.abs(nezhaV2.readRelAngle(this.mi)) < grados) {
            this.follow_line(base_speed)
        }
        nezhaV2.stop(this.mi)
        nezhaV2.stop(this.md)
    }
    
    public follow_line_until_cross(base_speed: number = 50, channel: number = PlanetX_Basic.TrackbitStateType.Tracking_State_1) {
        /** 
        Sigue la línea hasta que el sensor detecte un cruce en el canal indicado.
        Por defecto usa TRACKING_STATE_1 (todos los sensores activos = cruce).
        
 */
        PlanetX_Basic.Trackbit_get_state_value()
        while (!PlanetX_Basic.TrackbitState(channel)) {
            this.follow_line(base_speed)
            PlanetX_Basic.Trackbit_get_state_value()
        }
        nezhaV2.stop(this.mi)
        nezhaV2.stop(this.md)
    }
    
}

Robot.__initRobot()

//  ============== Código
let robot = new Robot(56, 120, mA, mB)
robot.mover(400, 70)
robot.girar(-120, 30, "both")
robot.mover(-400, 70)
