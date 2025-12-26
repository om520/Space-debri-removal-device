\documentclass{article}
\usepackage[utf8]{inputenc}
\usepackage{geometry}
\geometry{a4paper, margin=1in}
\usepackage{tikz}
\usepackage{graphicx}

% --- REQUIRED LIBRARIES ---
\usetikzlibrary{positioning, fit, backgrounds, shapes, arrows.meta, decorations.pathmorphing}

\begin{document}

\begin{figure}[htbp]
\centering
\resizebox{\textwidth}{!}{%
\begin{tikzpicture}[
    node distance=1.2cm and 1.8cm,
    font=\sffamily\footnotesize,
    >={Latex[length=1.6mm]},
    % ---------- Base styles ----------
    base/.style={
        rectangle, rounded corners,
        draw=black!70, align=center,
        inner sep=5pt
    },
    avionics/.style={base, fill=cyan!15, text width=3.0cm},
    eps/.style={base, fill=yellow!20, text width=3.0cm},
    prop/.style={base, fill=orange!20, text width=3.0cm},
    payload/.style={base, fill=red!15, text width=3.3cm},
    ext/.style={
        ellipse, draw=black, thick,
        fill=gray!10, text width=3.2cm,
        align=center
    },
    % ---------- Link styles ----------
    power/.style={->, draw=orange!80!black, line width=1.5pt},
    data/.style={->, draw=blue!60!black, dashed, thick},
    phys/.style={<->, draw=black!70, thick, dotted},
    rf/.style={<->, draw=black, decorate,
        decoration={snake, amplitude=.4mm, segment length=2mm}}
]

% ================= EXTERNAL (TOP) =================
\node[ext] (ground) {Ground Station\\ \textbf{TT\&C / Avoidance}};

% ================= AVIONICS =================
\node[avionics, below=1.8cm of ground] (ttc)
    {TT\&C Transceiver\\ \textbf{Communications}};
\node[avionics, right=of ttc] (sensors)
    {Perception Suite\\ \textbf{LiDAR / Camera / Radar}};
\node[avionics, below=of ttc] (obc)
    {Onboard Computer\\ \textbf{GNC \& Trajectory}};
\node[avionics, right=of obc] (ai)
    {Edge AI Module\\ \textbf{CNN / DNN}};

% ================= EPS =================
\node[eps, xshift=-6.5cm, below=1.8cm of obc] (ppu)
    {Power Processing Unit\\ \textbf{HV Regulation}};
\node[eps, above=of ppu] (solar)
    {Solar Array\\ \textbf{7.3 kW}};
\node[eps, below=of ppu] (batt)
    {Li-ion Battery\\ \textbf{Eclipse Region Power}};

% ================= PROPULSION =================
% Xenon / Main Thruster
\node[prop, xshift=6.5cm, below=1.8cm of ai] (ion)
    {Main Ion Thruster\\ \textbf{Retrograde Burn}};
\node[prop, above=of ion] (tank)
    {Xenon Propellant Tank\\ \textbf{20 kg}};

% RCS / New Tank
\node[prop, below=of ion] (rcs_tank)
    {RCS Fuel Tank\\ \textbf{3 kg}};
\node[prop, below=0.8cm of rcs_tank] (rcs)
    {RCS Thrusters\\ \textbf{Maneuvering Control}};

% ================= PAYLOAD =================
\node[payload, below=4.2cm of obc] (act)
    {Capture Actuation Unit\\ \textbf{Net \& Motors}};
\node[payload, below=of act] (claw)
    {Mechanical Claw\\ \textbf{Clamp / Release}};

% ================= EXTERNAL (BOTTOM) =================
\node[ext, below=1.6cm of claw] (debris)
    {Target Debris Object};

% ================= GROUP BOXES =================
\begin{scope}[on background layer]
    \node[
        fit=(ttc)(sensors)(obc)(ai),
        draw=blue!50, dashed, rounded corners,
        fill=blue!5, inner sep=8pt,
        label={[blue!80]north west:\textbf{Avionics \& GNC}}
    ] (grp_av) {};

    \node[
        fit=(solar)(ppu)(batt),
        draw=orange!60, dashed, rounded corners,
        fill=yellow!5, inner sep=8pt,
        label={[orange!80]north west:\textbf{Electrical Power System}}
    ] (grp_eps) {};

    \node[
        fit=(tank)(ion)(rcs_tank)(rcs),
        draw=red!60, dashed, rounded corners,
        fill=orange!5, inner sep=8pt,
        label={[red!80]north east:\textbf{Propulsion}}
    ] (grp_prop) {};

    \node[
        fit=(act)(claw),
        draw=red!60, dashed, rounded corners,
        fill=red!5, inner sep=8pt,
        label={[red!80]north west:\textbf{Capture Payload}}
    ] (grp_pay) {};

    \node[
        fit=(grp_av)(grp_eps)(grp_prop)(grp_pay),
        draw=black, thick, rounded corners=12pt
    ] (spacecraft) {};
\end{scope}

% ================= CONNECTIONS =================
% RF
\draw[rf] (ground) -- node[right,font=\scriptsize]{CCSDS} (ttc);

% Data
\draw[data] (sensors) -- (ai);
\draw[data] (ai) -- (obc);
\draw[data,<->] (obc) -- (ttc);
\draw[data] (obc.south) -- ++(0,-1.8) -- (act.north);
% Route data to propulsion group
\draw[data] (obc.east) -| ++(0.5,-1) -- ++(2.5,0) |- (ion.west);

% Power
\draw[power] (solar) -- (ppu);
\draw[power,<->] (batt) -- (ppu);
\draw[power] (ppu.east) -- ++(1.4,0) |- (ion.west);
\draw[power] (ppu.east) -- ++(1.4,0) |- (rcs.west);

% Physical (Propellant & Capture)
\draw[power] (tank) -- (ion);
\draw[power] (rcs_tank) -- (rcs);
\draw[phys] (act) -- (claw);
\draw[phys, thick] (claw) -- node[right,font=\scriptsize]{Capture} (debris);

% ================= LEGEND =================
\node[
    draw=black!50,
    fill=white,
    rounded corners,
    anchor=south east,
    minimum width=2.6cm,
    minimum height=1.2cm
] (legend) at ([xshift=-6pt,yshift=6pt]spacecraft.south east) {};

\node[
    anchor=north east,
    font=\scriptsize,
    align=left
] at (legend.north east)
{Power / Fuel \\ Data / Command};

\draw[power]
    ([xshift=0.3cm,yshift=-0.4cm]legend.north west) -- ++(0.7,0);
\draw[data]
    ([xshift=0.3cm,yshift=-0.8cm]legend.north west) -- ++(0.7,0);

\end{tikzpicture}%
}
\caption{System Architecture of the Space Debris Removal Module.}
\label{fig:architecture}
\end{figure}

\end{document}
