#Team 4 Sizing

import numpy as np
import ZachAircraftMath as zair
import ZachMath as zmath


"""
Args:
Wo: Takeoff weight [lbs]
PW: Power loading
WS: Wing loading[lb/ft2]
AR: aspect ratio
v_c: cruise velocity [m/s]
PW_ICE: Conventional power denisty [kW/kg]
PW_EM: Electrical power density [kW/kg]

Return: Empty Weight Fraction
Raymer Chp 6
"""
def emptyWeightFraction(Wo, PW, WS, AR, v_c, DoH=0, PW_ICE=2.3, PW_EM=4.2):
    PW_ICE_imp = PW_ICE*1.34102/2.20462
    PW_EM_imp = PW_EM*1.34102/2.20462
    v_c_fps = v_c*3.2808
    a = 0.37
    b = 0.09
    C1 = -0.06
    C2 = 0.08
    C3 = 0.08
    C4 =  -0.05
    C5 =  0.30
    weight = Wo*(a + b*np.power(Wo, C1) * np.power(AR, C2) * np.power(PW, C3) * np.power(WS, C4) * np.power(v_c_fps*0.5924838, C5))

    weight -= 2.575*(PW*Wo/PW_ICE_imp)**0.9
    engine_weight = PW*Wo*(1-DoH)/PW_ICE_imp + PW*Wo*DoH/PW_EM_imp
    weight += 2.575*(engine_weight)**0.9

    return weight/Wo
#!emptyWeightFraction



"""
Args:
Wo: Takeoff weight [lb]
LD: Lift to drag ratio
v_c: cruise speed [m/s]
R: range [km]
E: endurance [min]
n_StT: Shaft to thrust efficiency
BSFC: Specific fuel consumption [g/kW/h]

Returns: Fuel weight based on historical data [lb]
Raymer Chp 6
"""
def fuelWeight(Wo, LD, v_c, R, E, BSFC, n_StT=0.8, verbose=False):
    v_c_fps = v_c*3.2808
    R_m = R*1000
    E_s = E*60
    SFC_m = zair.BSFCtoInvMeters(BSFC)

    #Engine start, taxi, takeoff
    Wfrac_1 = 0.97
    #Climb and accelerate
    Wfrac_2 = 1.0065 - 0.0325*(v_c_fps/1125.32)
    #Cruise
    R_m = R*1000
    Wfrac_3 = np.exp(-1*R_m*SFC_m/(n_StT*LD))
    #Loiter
    V_ms = v_c_fps*0.3048
    Wfrac_4 = np.exp(-1*E_s*V_ms*SFC_m/(n_StT*LD))
    #Descent
    Wfrac_5 = 0.99
    #Landing and Taxi Back
    Wfrac_6 = 0.992
    
    #Mission segment weights
    Wf_1 = (1 - Wfrac_1) * Wo
    Wf_2 = (1 - Wfrac_2) * (Wo - Wf_1)
    Wf_3 = (1 - Wfrac_3) * (Wo - Wf_1 - Wf_2)
    Wf_4 = (1 - Wfrac_4) * (Wo - Wf_1 - Wf_2 - Wf_3)
    Wf_5 = (1 - Wfrac_5) * (Wo - Wf_1 - Wf_2 - Wf_3 - Wf_4)
    Wf_6 = (1 - Wfrac_6) * (Wo - Wf_1 - Wf_2 - Wf_3 - Wf_4 - Wf_4)
    if verbose:
        print("Takeoff Weight: %.2f lbs" % Wf_1)
        print("Climb Weight: %.2f lbs" % Wf_2)
        print("Cruise Weight: %.2f lbs" % Wf_3)
        print("Loiter Weight: %.2f lbs" % Wf_4)
        print("Descent Weight: %.2f lbs" % Wf_5)
        print("Landing Weight: %.2f lbs" % Wf_6)
    return 1.06*(Wf_1 + Wf_2 + Wf_3 + Wf_4 + Wf_5 + Wf_6)
#!fuelWeight



"""
Args: 
Wfixed: fixed weight [lb]
Wfuel: Fuel/battery weight [lb]
WeFrac: Empty weight fractio

Returns: maximum takeoff weight
"""
def calculateMTOW(Wfixed, Wfuel, WeFrac):
    return ((Wfixed + Wfuel)/(1-WeFrac))
#!calculateMTOW



"""
Args: 
Wguess: Initial guess sizing [lb]
Wfixed: Fixed weight [lb]
PW: Power to weight ratio
WS: Wing loading [lb/ft2]
AR: Aspect ratio
C_Dmin: Minimum drag coefficient
alt_c: Cruise altitude [m]
v_c: Cruise speed [m/s]
R: Rnage [km]
E: Endurance [min]
n_StT: Shaft to thrust efficiency
sfc: Spefici fuel consumption [god knows]
verbose: Will print out info regarding the weights

Returns: maximum takeoff weight [lb]
"""
def findMTOW(Wguess, Wfixed, PW, WS, AR, C_Dmin, alt_c, v_c, R, E, BSFC, n_StT=0.8, verbose=False):
    LD = zair.liftDragRatio(WS, AR, C_Dmin, alt_c, v_c)

    Weight1 = Wguess
    Wfuel = fuelWeight(Weight1, LD, v_c, R, E, BSFC, n_StT)
    WeFrac = emptyWeightFraction(Weight1, PW, WS, AR, v_c)
    Weight2 = calculateMTOW(Wfixed, Wfuel, WeFrac)
    while (np.absolute((Weight2-Weight1)/Weight1) > 0.001):
        Weight1 = Weight2;
        Wfuel = fuelWeight(Weight1, LD, v_c, R, E, BSFC, n_StT)
        WeFrac = emptyWeightFraction(Weight1, PW, WS, AR, v_c)
        Weight2 = calculateMTOW(Wfixed, Wfuel, WeFrac)
        
    if (verbose == True):
        print("MTOW: ", Weight2, "lbs")
        print("Empty Weight: ", WeFrac*Weight2, "lbs")
        print("Fuel Weight: ", Wfuel, "lbs")
        print("Fixed Weight: ", Wfixed, "lbs")

    return Weight2
#!findMTOW



"""
Args:
AR: Aspect ratio
TOP: Take off parameter (~250)
v_c: Cruise speed [m/s]
alt_c: Cruise alt [m]
v_climb: Climb rate [m/s]
alt_sc: Service ceiling [m]
C_Dmin: Minimum drag coefficient
C_LTO: Takeoff lift coefficient
bankAngle: Maximum bank angle for sustained turn [deg]
n_StT: Shaft to thrust efficiency 
sigma_TO: HotandHeavy takeoff desnity [slugs/ft3] [SOMEWHAT TEMPORARY]
verbose: Prints out info if True

Returns: Design point (W/S, P/W) based on varibales above, returns 0,0 if no point is found
"""
def Constraint(AR, TOP, v_c, alt_c, v_climb, alt_sc, C_Dmin, C_LTO, bankAngle, n_StT=0.8, sigma_TO=0.71806, verbose=False):
    
    k = zair.dragFactor(AR)
    q_imp = 0.02089*zair.dynamicPressure(alt_c, v_c) #q in lb/ft^2
    v_c_fps = v_c*3.2808
    v_climb_fps = v_climb*3.2808
    rho_ceiling_imp = 0.02089*zair.altitudeToDensity(alt_sc)
    loadFactor = np.cos(bankAngle*np.pi/180) #Load factor
    
    x_i = 5    #Starting W/S
    x_f = 70   #Final W/S
    x_d = 5    #Delta W/S
    
    c_Takeoff = lambda x: x/(TOP*C_LTO*sigma_TO)                                                                #Hot and High Takeoff
    c_Climb = lambda x: (v_climb_fps/v_c_fps) + (q_imp*C_Dmin/x) + (k*x/q_imp)                                      #Climb constraint
    c_Speed = lambda x: (q_imp*C_Dmin/x) + (k*x/q_imp)                                                          #Cruise speed constraint
    c_Ceiling = lambda x: (1.667/np.sqrt(2*x*np.sqrt(k/(3*C_Dmin))/rho_ceiling_imp)) + (4*np.sqrt(k*C_Dmin/3))  #Service ceiling constraint
    c_Turn = lambda x: q_imp*((C_Dmin/x) + (k*np.power(loadFactor/q_imp, 2)*x))                                 #Constant turn constraint

    #Find the initial max function
    prevMax = zmath.FindMax(c_Takeoff, zmath.FindMax(c_Climb, zmath.FindMax(c_Speed, zmath.FindMax(c_Ceiling, c_Turn, x_i), x_i), x_i), x_i)
    
    #Loop through values of W/S
    for x in range(x_i, x_f+x_d, x_d):
        #Find the new maximum function
        currMax = zmath.FindMax(c_Takeoff, zmath.FindMax(c_Climb, zmath.FindMax(c_Speed, zmath.FindMax(c_Ceiling, c_Turn, x), x), x), x)
        #Check if the maximum function has changed
        if(currMax != prevMax):
            WS, TW = zmath.FindIntersection(currMax, prevMax, x-x_d, x+x_d)
            PW = TW*v_c_fps/(550*n_StT)
            if verbose:
                print("W/S: ", WS, " lb/ft^2")
                print("P/W: ", PW)
            return WS,PW
        
        prevMax = currMax

    if verbose:
        print("Failed to find point")
    return 0,0 #Fail case
#!Constraint