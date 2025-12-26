```mermaid
graph TD
    %% --- External Entities ---
    Ground[("📡 Earth-Based Station<br/>(Global Avoidance/TT&C)")]
    Debris((Target Debris))

    %% --- Spacecraft Bus ---
    subgraph Spacecraft [Space Debris Removal Module]
        
        %% --- Subsystem: Power (EPS) ---
        subgraph EPS [Power Subsystem]
            Solar[("☀️ Solar Panels<br/>(7.3 kW)")]
            PPU["Power Processing Unit<br/>(High Voltage Regulation)"]
            Batt[("🔋 Li-ion Battery<br/>(33 kg / Eclipse Ops)")]
            
            Solar --> PPU
            Batt <--> PPU
        end

        %% --- Subsystem: Avionics & GNC ---
        subgraph Avionics [Avionics & GNC]
            OBC["Onboard Computer<br/>(Trajectory & Control)"]
            AI_Edge["Edge Inference Engine<br/>(CNN/DNN)"]
            
            %% Sensors
            Sensors_Perc["Perception Sensors<br/>(Camera / Radar / LiDAR)"]
            
            %% Comms
            Transceiver[TT&C Transceiver]
            
            Sensors_Perc --> AI_Edge
            AI_Edge --> OBC
            OBC <--> Transceiver
        end

        %% --- Subsystem: Propulsion ---
        subgraph Propulsion [Propulsion System]
            Tank[("⛽ Xenon Tank<br/>(20 kg)")]
            Ion[("🚀 Main Ion Thruster<br/>(Back / Retrograde Thrust)")]
            RCS[("💨 Side Thrusters<br/>(Maneuvering / Avoidance)")]
            
            Tank --> Ion
            Tank --> RCS
        end

        %% --- Subsystem: Capture Payload ---
        subgraph Payload [Capture Mechanism]
            Actuators[Net & Claw Actuator]
            Claw[("🪝 Mechanical Claw<br/>(Clamp & Declamp)")]
            
            Actuators --> Claw
        end

        %% --- System Interconnections ---
        %% Power Distribution (Thick Lines)
        PPU ==>|High Voltage| Ion
        PPU ==>|Power| RCS
        PPU ==>|Power| Avionics
        PPU ==>|Power| Payload

        %% Data & Control Flow (Dotted Lines)
        OBC -.->|Thrust Cmd| Ion
        OBC -.->|Maneuver Cmd| RCS
        OBC -.->|Capture Cmd| Actuators
    end

    %% --- External Links ---
    Ground <-->|CCSDS Link| Transceiver
    Claw -.->|Physical Capture| Debris
    
    %% Styling
    classDef power fill:#ffdd00,stroke:#333,stroke-width:2px,color:black;
    classDef prop fill:#ff9900,stroke:#333,stroke-width:2px,color:black;
    classDef avionics fill:#00ccff,stroke:#333,stroke-width:2px,color:black;
    classDef payload fill:#ff5555,stroke:#333,stroke-width:2px,color:black;
    classDef ext fill:#eeeeee,stroke:#333,stroke-width:1px,color:black;
    
    class EPS,Solar,PPU,Batt power;
    class Propulsion,Ion,RCS,Tank prop;
    class Avionics,OBC,AI_Edge,Sensors_Perc,Transceiver avionics;
    class Payload,Claw,Actuators payload;
    class Ground,Debris ext;
