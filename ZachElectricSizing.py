#Team 4 Electric Sizing

import numpy as np
import ZachAircraftMath as zair
import ZachMath as zmath
import ZachAircraftSizing as zize


"""
Args:
MTOW: Maximum takeoff weight [lbs]
WS: Wing loading [lb/ft^2]
AR: Aspect ratio
LD: Lift to drag ratio
DoH: Degree of hybridization

v_TO: Takeoff speed [m/s]
v_c: Cruise speed [m/s]
v_climb: Rate of climb [m/s]
ddp: Deep discharge protection ratio
tf_fraction: Trapped fuel fraction
n_BtT: Battery to thrust efficiency
E_B: Battery specific energy [Wh/kg]
BSFC: Brake specific fuel consumption [g/kWh]
R: Range [km]
E: Endurance [min]
alt_c: Cruise altitude [m]
c_Dmin: Mininium drag coefficient
c_L0: Lift coefficient
verbose: Setting verbose to true prints all the segment weights

Returns: Fuel weight [kg]
"""
def energyMass(MTOW, WS, AR, LD, DoH, v_TO, v_c, v_climb, R, E, alt_c, c_Dmin, c_L0, 
               E_B=1000, BSFC=280, ddp=0.2, tf_fraction=0.06, n_BtT=0.95, n_StT=0.8, 
               verbose=False):

    WS_metric = WS*4.88243*9.81 #Gets WS in N/m^2
    BSFC_metric = BSFC/(3.6E9)

    aircraft_mass = MTOW*0.453592
    dE, dE_B, dE_F, dt, dm_B, dm_F = 0, 0, 0, 0, 0, 0 #Change in energy, battery energy, fuel energy, time, battery mass, fuel mass
    totalMass_B = 0 #Total battery mass
    totalMass_F = 0 #Total fuel mass
    

    """
    Take-off
    """
    segment_mass_B,segment_mass_F = 0,0

    weight_fraction = 0.97 #From Raymer p150

    dE = aircraft_mass*(1-weight_fraction)*n_StT/((1+tf_fraction)*BSFC_metric)

    dE_B = dE*DoH*0.25
    dE_F = dE-dE_B

    dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
    dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

    segment_mass_B += dm_B
    segment_mass_F += dm_F
    aircraft_mass -= dm_F

    if (verbose):
        print("Takeoff Segment: B(%.2f lbs) F(%0.2f lbs)" % (2.2046*segment_mass_B, 2.2046*segment_mass_F))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    """
    Climb
    """
    segment_mass_B,segment_mass_F = 0,0

    current_alt = 0
    dt = 10 #10 second delta t
    v_prev = v_TO
    while (current_alt < alt_c):
        v_y = zair.findVelocityRoC(WS=WS, alt=current_alt, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR, LD=LD)
        v = (v_y**2 + v_climb**2)**0.5
        dv = v - v_prev

        dE_drag = aircraft_mass*9.81*v*dt/LD
        dE_kin = 0.5*aircraft_mass*dv*np.absolute(dv)
        dE_pot = aircraft_mass*9.81*v_climb*dt
        dE = dE_drag + dE_kin + dE_pot

        dE_B = dE*DoH
        dE_F = dE-dE_B

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        v_prev = v
        current_alt += v_climb*dt

    if (verbose):
        print("Climb Segment: B(%.2f lbs) F(%0.2f lbs)" % (2.2046*segment_mass_B, 2.2046*segment_mass_F))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    """
    Cruise
    Assumes plane maintains a constant speed and flight level
    """
    segment_mass_B,segment_mass_F = 0,0

    dt=10
    distance = 0 #Distance travelled in m
    while (distance < R*1000):
        dE_drag = aircraft_mass*9.81*v_c*dt/LD
        dE_kin = 0
        dE_pot = 0
        dE = dE_drag + dE_kin + dE_pot

        dE_B = dE*DoH
        dE_F = dE-dE_B

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        distance += (dt*v_c)

    if (verbose):
        print("Cruise Segment: B(%.2f lbs) F(%0.2f lbs)" % (2.2046*segment_mass_B, 2.2046*segment_mass_F))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    """
    Loiter
    """
    segment_mass_B,segment_mass_F = 0,0

    dt = 10
    t_loiter = 0
    v = zair.findVelocityLD(WS=WS, alt=alt_c, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR)
    dv = v - v_c
    while (t_loiter < E*60):
        dE_drag = aircraft_mass*9.81*v*dt/LD
        dE_kin = 0.5*aircraft_mass*9.81*dv*np.absolute(dv)
        dE_pot = 0

        dE_B = dE*DoH
        dE_F = dE-dE_B

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        t_loiter += dt
        dv = 0

    if (verbose):
        print("Loiter Segment: B(%.2f lbs) F(%0.2f lbs)" % (2.2046*segment_mass_B, 2.2046*segment_mass_F))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F
    
    
    """
    Decsent
    """
    segment_mass_B,segment_mass_F = 0,0

    current_alt = alt_c
    dt = 10 #10 second delta t
    v_prev = zair.findVelocityLD(WS=WS, alt=alt_c, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR)
    while (current_alt > 0):
        v_y = zair.findVelocityRoC(WS=WS, alt=current_alt, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR, LD=LD)
        v_descent = v_y * 0.0524 #Based on a 3deg angle of approach
        v = (v_y**2 + v_descent**2)**0.5
        dv = v - v_prev

        dE_drag = aircraft_mass*9.81*v*dt/LD
        dE_kin = 0.5*aircraft_mass*dv*np.absolute(dv)
        dE_pot = -1*aircraft_mass*9.81*v_descent*dt
        dE = dE_drag + dE_kin + dE_pot

        dE_B = dE*DoH
        dE_F = dE-dE_B

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        v_prev = v
        current_alt -= v_descent*dt

    if (verbose):
        print("Decsent Segment: B(%.2f lbs) F(%0.2f lbs)" % (2.2046*segment_mass_B, 2.2046*segment_mass_F))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F

    """
    Landing
    """
    segment_mass_B,segment_mass_F = 0,0

    weight_fraction = 0.992 #From Raymer p153

    dE = aircraft_mass*(1-weight_fraction)*n_StT/((1+tf_fraction)*BSFC_metric)
    dE_B = dE*DoH*0.25
    dE_F = dE-dE_B

    dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
    dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

    segment_mass_B += dm_B
    segment_mass_F += dm_F
    aircraft_mass -= dm_F

    if (verbose):
        print("Landing Segment: B(%.2f lbs) F(%0.2f lbs)" % (2.2046*segment_mass_B, 2.2046*segment_mass_F))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    return totalMass_B*2.20462, totalMass_F*2.20462
#!energyMass



"""
Args: 
Wguess: Initial guess sizing [lb]
Wfixed: Fixed weight [lb]
DoH: Degree of hybridization (1=Fully electric, 0=Conventional)
PW: Power to weight ratio
WS: Wing loading [lb/ft2]
AR: Aspect ratio
c_Dmin: Minimum drag coefficient
c_L0: Lift coefficient
alt_c: Cruise altitude [m]
v_c: Cruise speed [m/s]
v_TO: Takeoff velocity [m/s]
v_climb: Climb speed [m/s]
R: Rnage [km]
E: Endurance [min]
n_StT: Shaft to thrust efficiency
BSFC: Spefici fuel consumption [g/kW/h]
E_B: Battery specific energy [Wh/kg]
verbose: Will print out info regarding the weights

Returns: maximum takeoff weight [lb]
"""
def findMTOW_EnergyModel(Wguess, Wfixed, DoH, PW, WS, AR, c_Dmin, c_L0, alt_c, v_c, v_TO, 
                        v_climb, R, E, BSFC, E_B, n_StT=0.8, n_BtT=0.95, verbose=False):

    LD = zair.liftDragRatio(WS, AR, c_Dmin, alt_c, v_c)

    Weight1 = Wguess

    m_B, m_F = energyMass(Weight1, WS=WS, AR=AR, LD=LD, DoH=DoH, v_TO=v_TO, v_c=v_c, 
        v_climb=v_climb, ddp=0.2, tf_fraction=0.06, n_BtT=n_BtT, n_StT=n_StT, E_B=E_B, 
        BSFC=BSFC, R=R, E=E, alt_c=alt_c, c_Dmin=c_Dmin, c_L0=c_L0, verbose=False)
    Wenergy = m_B + m_F

    WeFrac = zize.emptyWeightFraction(Weight1, PW, WS, AR, v_c, DoH=DoH)
    Weight2 = zize.calculateMTOW(Wfixed, Wenergy, WeFrac)
    while (np.absolute((Weight2-Weight1)/Weight1) > 0.001):
        Weight1 = Weight2;

        m_B, m_F = energyMass(Weight1, WS=WS, AR=AR, LD=LD, DoH=DoH, v_TO=v_TO, v_c=v_c, 
            v_climb=v_climb, ddp=0.2, tf_fraction=0.06, n_BtT=n_BtT, n_StT=n_StT, E_B=E_B, 
            BSFC=BSFC, R=R, E=E, alt_c=alt_c, c_Dmin=c_Dmin, c_L0=c_L0, verbose=False)
        Wenergy = m_B + m_F

        WeFrac = zize.emptyWeightFraction(Weight1, PW, WS, AR, v_c, DoH=DoH)
        Weight2 = zize.calculateMTOW(Wfixed, Wenergy, WeFrac)
        
    if (verbose == True):
        print("MTOW: ", Weight2, "lbs")
        print("Empty Weight: ", WeFrac*Weight2, "lbs")
        print("Energy Weight: ", Wenergy, "lbs")
        print("Fixed Weight: ", Wfixed, "lbs")

    return Weight2
#!findMTOW



"""
Args:
Hybrid Type: 'S' Series, 'P' Parallel
AR: Aspect ratio
TOP: Take off parameter (~250)
v_c: Cruise speed [m/s]
alt_c: Cruise alt [m]
v_climb: Climb rate [m/s]
alt_sc: Service ceiling [m]
c_Dmin: Minimum drag coefficient
C_LTO: Takeoff lift coefficient
bankAngle: Maximum bank angle for sustained turn [deg]
n_StT: Shaft to thrust efficiency 
sigma_TO: HotandHeavy takeoff desnity [slugs/ft3] [SOMEWHAT TEMPORARY]
verbose: Prints out info if True

Returns: (PW_EM, PW_ICE) 
            PW_EM: Electric motor power to weight [hp/lb]
            PW_ICE: Conventional engine power to weight [hp/lb]
"""
def designPoint_Hybrid(HybridType, WS, AR, TOP, v_c, alt_c, v_climb, alt_sc, c_Dmin, c_LTO, bankAngle, 
    n_StT=0.8, sigma_TO=0.71806, verbose=False):
    
    #Find imperial equivilants and some other values
    k = zair.dragFactor(AR)                                  #Lift induced drag factor k
    q_imp = 0.02089*zair.dynamicPressure(alt_c, v_c)         #q in lb/ft^2
    v_c_fps = v_c*3.2808                                     #v_c in fps
    v_climb_fps = v_climb*3.2808                             #v_climb in fps
    rho_ceiling_imp = 0.02089*zair.altitudeToDensity(alt_sc) #Service ceiling in slug/ft^3
    loadFactor = np.cos(bankAngle*np.pi/180)                 #Load factor

    #Constraints
    #EM constraints
    c_Takeoff = lambda x: x/(TOP*c_LTO*sigma_TO) #Hot and High Takeoff
    c_Climb = lambda x: (v_climb_fps/v_c_fps) + (q_imp*c_Dmin/x) + (k*x/q_imp) #Climb constraint
    #ICE constraints                                            
    c_Speed = lambda x: (q_imp*c_Dmin/x) + (k*x/q_imp) #Cruise speed constraint
    c_Ceiling = lambda x: (1.667/np.sqrt(2*x*np.sqrt(k/(3*c_Dmin))/rho_ceiling_imp)) + (4*np.sqrt(k*c_Dmin/3)) #Service ceiling constraint
    c_Turn = lambda x: q_imp*((c_Dmin/x) + (k*np.power(loadFactor/q_imp, 2)*x)) #Constant level turn constraint

    #Find max EM constraint
    max_func = zmath.FindMax(c_Takeoff, c_Climb, WS)
    TW_EM = max_func(WS)

    #Find max ICE constraint
    max_func = zmath.FindMax(c_Speed, zmath.FindMax(c_Ceiling, c_Turn, WS), WS)
    TW_ICE = max_func(WS)

    #Convert from TW to PW
    PW_EM = TW_EM*v_c_fps/(550*n_StT)
    PW_ICE = TW_ICE*v_c_fps/(550*n_StT)

    #Convert to parallel
    if (HybridType=='S'):
        pass
    elif (HybridType=='P'):
        PW_EM -= PW_ICE
    else:
        raise ValueError("Use P or H for hybrid type.")

    if verbose:
        print("EM PW: %.3f hp/lb" % PW_EM)
        print("ICE PW: %.3f hp/lb" % PW_ICE)

    return PW_EM, PW_ICE
#!designPoint_Hybrid



"""
Args:
Wo: Takeoff weight [lbs]
PW_EM: Required electric motor power loading
PW_ICE: Rrequired conventional engine power loading
WS: Wing loading[lb/ft2]
AR: aspect ratio
v_c: cruise velocity [m/s]
PW_ICE_avg: Conventional power denisty [kW/kg]
PW_EM_avg: Electrical power density [kW/kg]

Return: Empty Weight Fraction
Raymer Chp 6
"""
def emptyWeightFraction_Hybrid(Wo, PW_EM, PW_ICE, WS, AR, v_c, PW_ICE_avg=2.3, PW_EM_avg=4.2):
    PW_ICE_imp = PW_ICE_avg*1.34102/2.20462 #Convert from kW/kg to hp/lb
    PW_EM_imp = PW_EM_avg*1.34102/2.20462 #Convert from kW/kg to hp/lb
    v_c_fps = v_c*3.2808 #Convert from m/s to fps
   
    Wo_kg = zair.lb_to_kg(Wo)
    weight_empty = zair.kg_to_lb(0.743*(Wo_kg**0.94))*((1+0.574)/2)

    engine_weight = (PW_ICE/PW_ICE_imp + PW_EM/PW_EM_imp)*Wo #Find actual erngine weight
    weight_empty += 2.575*(engine_weight)**0.9 #Add actual engine weight

    return weight_empty/Wo #Return empty weight fraction
#!emptyWeightFraction



"""
Args:
Hybrid Type: 'S' Series, 'P' Parallel, 'C' Conventional
MTOW: Maximum takeoff weight [lbs]
WS: Wing loading [lb/ft^2]
AR: Aspect ratio
LD: Lift to drag ratio
DoH: Degree of hybridization

v_TO: Takeoff speed [m/s]
v_c: Cruise speed [m/s]
v_climb: Rate of climb [m/s]
ddp: Deep discharge protection ratio
tf_fraction: Trapped fuel fraction
n_BtT: Battery to thrust efficiency
E_B: Battery specific energy [Wh/kg]
BSFC: Brake specific fuel consumption [g/kWh]
R: Range [km]
E: Endurance [min]
alt_c: Cruise altitude [m]
c_Dmin: Mininium drag coefficient
c_L0: Lift coefficient

b_ff: Battery fudge factor (1=No change, 0=No energy usage)
f_ff: Fuel fudge factor (1=No change, 0=No energy usage)

verbose: Setting verbose to true prints all the segment weights

Returns: Fuel weight [kg]
"""
def energyMass_Hybrid(HybridType, MTOW, WS, AR, PW_EM, PW_ICE, LD, v_TO, v_c, v_climb, R, E, alt_c, c_Dmin, c_L0, 
               E_B, BSFC, ddp=0.2, tf_fraction=0.06, n_BtT=0.95, n_StT=0.8, 
               b_ff=1, f_ff=1, verbose=False):

    WS_metric = WS*4.88243*9.81 #Gets WS in N/m^2
    BSFC_metric = BSFC/(3.6E9) #Gets SFC in J/kg

    aircraft_mass = zair.lb_to_kg(MTOW)
    dE, dE_B, dE_F, dt, dm_B, dm_F = 0, 0, 0, 0, 0, 0 #Change in energy, battery energy, fuel energy, time, battery mass, fuel mass
    totalMass_B = 0 #Total battery mass
    totalMass_F = 0 #Total fuel mass
    
    #Fuel and battery split during takeoff and climb ranges from 0 to 1
    if (HybridType=='S'):
        fuel_perc = PW_ICE/PW_EM
        bat_perc = (PW_EM-PW_ICE)/PW_EM
    elif (HybridType=='P'):
        fuel_perc = PW_ICE/(PW_EM+PW_ICE)
        bat_perc = PW_EM/(PW_EM+PW_ICE)
    elif (HybridType == 'C'):
        fuel_perc = 1
        bat_perc = 0
    else:
        raise ValueError("Use P, H, or C for hybrid type.")


    """
    Take-off
    """
    segment_mass_B,segment_mass_F = 0,0

    weight_fraction = 0.97 #From Raymer p150

    dE = aircraft_mass*(1-weight_fraction)*n_StT/((1+tf_fraction)*BSFC_metric)

    dE_B = dE*bat_perc*b_ff
    dE_F = dE*fuel_perc*f_ff

    dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
    dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

    segment_mass_B += dm_B
    segment_mass_F += dm_F
    aircraft_mass -= dm_F

    if (verbose):
        print("Takeoff Segment: B(%.2f lbs) F(%0.2f lbs)" % (zair.kg_to_lb(segment_mass_B), zair.kg_to_lb(segment_mass_F)))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F

    """
    Climb
    """
    segment_mass_B,segment_mass_F = 0,0

    current_alt = 0
    dt = 10 #10 second delta t
    v_prev = v_TO
    while (current_alt < alt_c):
        v_y = zair.findVelocityRoC(WS=WS, alt=current_alt, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR, LD=LD)
        v = (v_y**2 + v_climb**2)**0.5
        dv = v - v_prev

        dE_drag = aircraft_mass*9.81*v*dt/LD
        dE_kin = 0.5*aircraft_mass*dv*np.absolute(dv)
        dE_pot = aircraft_mass*9.81*v_climb*dt
        dE = dE_drag + dE_kin + dE_pot

        dE_B = dE*bat_perc*b_ff
        dE_F = dE*fuel_perc*f_ff

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        v_prev = v
        current_alt += v_climb*dt

    if (verbose):
        print("Climb Segment: B(%.2f lbs) F(%0.2f lbs)" % (zair.kg_to_lb(segment_mass_B), zair.kg_to_lb(segment_mass_F)))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    """
    Cruise
    Assumes plane maintains a constant speed and flight level
    """
    segment_mass_B,segment_mass_F = 0,0

    dt=10
    distance = 0 #Distance travelled in m
    while (distance < R*1000):
        dE_drag = aircraft_mass*9.81*v_c*dt/LD
        dE_kin = 0
        dE_pot = 0
        dE = dE_drag + dE_kin + dE_pot

        dE_B = 0*b_ff
        dE_F = dE*f_ff

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        distance += (dt*v_c)

    if (verbose):
        print("Cruise Segment: B(%.2f lbs) F(%0.2f lbs)" % (zair.kg_to_lb(segment_mass_B), zair.kg_to_lb(segment_mass_F)))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    """
    Loiter
    """
    segment_mass_B,segment_mass_F = 0,0

    dt = 10
    t_loiter = 0
    v = zair.findVelocityLD(WS=WS, alt=alt_c, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR)
    dv = v - v_c
    while (t_loiter < E*60):
        dE_drag = aircraft_mass*9.81*v*dt/LD
        dE_kin = 0.5*aircraft_mass*9.81*dv*np.absolute(dv)
        dE_pot = 0

        dE_B = 0*b_ff
        dE_F = dE*f_ff

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        t_loiter += dt
        dv = 0

    if (verbose):
        print("Loiter Segment: B(%.2f lbs) F(%0.2f lbs)" % (zair.kg_to_lb(segment_mass_B), zair.kg_to_lb(segment_mass_F)))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F
    
    
    """
    Decsent
    """
    segment_mass_B,segment_mass_F = 0,0

    current_alt = alt_c
    dt = 10 #10 second delta t
    v_prev = zair.findVelocityLD(WS=WS, alt=alt_c, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR)
    while (current_alt > 0):
        v_y = zair.findVelocityRoC(WS=WS, alt=current_alt, c_Dmin=c_Dmin, c_L0=c_L0, AR=AR, LD=LD)
        v_descent = v_y * 0.0524 #Based on a 3deg angle of approach
        v = (v_y**2 + v_descent**2)**0.5
        dv = v - v_prev

        dE_drag = aircraft_mass*9.81*v*dt/LD
        dE_kin = 0.5*aircraft_mass*dv*np.absolute(dv)
        dE_pot = -1*aircraft_mass*9.81*v_descent*dt
        dE = dE_drag + dE_kin + dE_pot

        dE_B = 0*b_ff
        dE_F = dE*f_ff

        dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
        dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

        segment_mass_B += dm_B
        segment_mass_F += dm_F
        aircraft_mass -= dm_F

        v_prev = v
        current_alt -= v_descent*dt

    if (verbose):
        print("Decsent Segment: B(%.2f lbs) F(%0.2f lbs)" % (zair.kg_to_lb(segment_mass_B), zair.kg_to_lb(segment_mass_F)))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F

    """
    Landing
    """
    segment_mass_B,segment_mass_F = 0,0

    weight_fraction = 0.992 #From Raymer p153

    dE = aircraft_mass*(1-weight_fraction)*n_StT/((1+tf_fraction)*BSFC_metric)

    dE_B = 0*b_ff
    dE_F = dE*f_ff

    dm_B = (1+ddp)*dE_B/(n_BtT*E_B*3600)
    dm_F = (1+tf_fraction)*BSFC_metric*dE_F/n_StT

    segment_mass_B += dm_B
    segment_mass_F += dm_F
    aircraft_mass -= dm_F

    if (verbose):
        print("Landing Segment: B(%.2f lbs) F(%0.2f lbs)" % (zair.kg_to_lb(segment_mass_B), zair.kg_to_lb(segment_mass_F)))

    totalMass_B += segment_mass_B
    totalMass_F += segment_mass_F


    return zair.kg_to_lb(totalMass_B), zair.kg_to_lb(totalMass_F)
#!energyMass_Hybrid


"""
Args: 
Wguess: Initial guess sizing [lb]
Wfixed: Fixed weight [lb]
DoH: Degree of hybridization (1=Fully electric, 0=Conventional)
PW: Power to weight ratio
WS: Wing loading [lb/ft2]
AR: Aspect ratio
c_Dmin: Minimum drag coefficient
c_L0: Lift coefficient
alt_c: Cruise altitude [m]
v_c: Cruise speed [m/s]
v_TO: Takeoff velocity [m/s]
v_climb: Climb speed [m/s]
R: Rnage [km]
E: Endurance [min]
n_StT: Shaft to thrust efficiency
BSFC: Spefici fuel consumption [g/kW/h]
E_B: Battery specific energy [Wh/kg]

b_ff: Battery fudge factor (1=No change, 0=No energy usage)
f_ff: Fuel fudge factor (1=No change, 0=No energy usage)

verbose: Will print out info regarding the weights

Returns: maximum takeoff weight [lb]
"""
def findMTOW_Hybrid(HybridType, Wguess, Wfixed, PW_EM, PW_ICE, WS, AR, c_Dmin, c_L0, alt_c, v_c, v_TO, 
                        v_climb, R, E, BSFC, E_B, n_StT=0.8, n_BtT=0.95, b_ff=1, f_ff=1, verbose=False):

    if (HybridType!='P' and HybridType!='S'):
        ValueError("Use P or H for hybrid type.")


    LD = 14#zair.liftDragRatio(WS, AR, c_Dmin, alt_c, v_c)

    Weight1 = Wguess

    m_B, m_F = energyMass_Hybrid(HybridType=HybridType, MTOW=Weight1, WS=WS, AR=AR, PW_EM=PW_EM, PW_ICE=PW_ICE, LD=LD,
                v_TO=v_TO, v_c=v_c, v_climb=v_climb, R=R, E=E, alt_c=alt_c, c_Dmin=c_Dmin, 
                c_L0=c_L0, E_B=E_B, BSFC=BSFC, ddp=0.2, tf_fraction=0.06, n_BtT=0.95, n_StT=0.8, 
                b_ff=b_ff, f_ff=f_ff, verbose=False)
    Wenergy = m_B + m_F

    WeFrac = emptyWeightFraction_Hybrid(Wo=Weight1, PW_EM=PW_EM, PW_ICE=PW_ICE, WS=WS, AR=AR,
                v_c=v_c, PW_ICE_avg=2.3, PW_EM_avg=4.2)

    Weight2 = zize.calculateMTOW(Wfixed, Wenergy, WeFrac)
    while (np.absolute((Weight2-Weight1)/Weight1) > 0.001):
        Weight1 = Weight2;

        m_B, m_F = energyMass_Hybrid(HybridType=HybridType, MTOW=Weight1, WS=WS, AR=AR, PW_EM=PW_EM, PW_ICE=PW_ICE, LD=LD,
                v_TO=v_TO, v_c=v_c, v_climb=v_climb, R=R, E=E, alt_c=alt_c, c_Dmin=c_Dmin, 
                c_L0=c_L0, E_B=E_B, BSFC=BSFC, ddp=0.2, tf_fraction=0.06, n_BtT=0.95, n_StT=0.8, 
                b_ff=b_ff, f_ff=f_ff, verbose=False)
        Wenergy = m_B + m_F

        WeFrac = emptyWeightFraction_Hybrid(Wo=Weight1, PW_EM=PW_EM, PW_ICE=PW_ICE, WS=WS, AR=AR,
                v_c=v_c, PW_ICE_avg=2.3, PW_EM_avg=4.2)

        Weight2 = zize.calculateMTOW(Wfixed, Wenergy, WeFrac)
        
    if (verbose == True):
        print("MTOW: %.2flbs" % Weight2)
        print("Empty Weight: %.2flbs" % (WeFrac*Weight2))
        print("Energy Weight: B(%.2flbs) F(%.2flbs) T(%.2flbs)" % (m_B, m_F, Wenergy))
        print("Fixed Weight: %.2flbs" % Wfixed)

    return Weight2
#!findMTOW




