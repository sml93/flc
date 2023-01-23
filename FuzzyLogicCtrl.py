import numpy as np

from ceilingEffect import thrustCE

""" 
Steps for fuzzy logic controller:
Step 1: Identification of variables
Step 2: Fuzzy subset config
Step 3: Obtaining Membership Functions (MF)
Step 4: Fuzzy Rule Base Configuration
Step 5: Normalizing and scaling factors
Step 6: Fuzzification
Step 7: Identification of output (op)
Step 8: Defuzzification
"""


class flc():
    def __init__(self, cdist, vuav):
        ## Defining fuzzy input variables
        self._ceiling_dist = cdist
        self._vel_uav = vuav

        ## Initialise all ceiling distance linguistic variables
        self.NLCD = 0
        self.NSCD = 0
        self.ZCD = 0
        self.PSCD = 0
        self.PLCD = 0

        ## Initialise all speed linguistic variables
        self.NLSD = 0
        self.NSSD = 0
        self.ZSD = 0
        self.PSSD = 0
        self.PLSD = 0

        ## Initialise all throttle/thrust setpoint linguistic variables
        self.NLTC = 0
        self.NSTC = 0
        self.ZTC = 0
        self.PSTC = 0
        self.PLTC = 0

        ## Defining default fuzzy output variables (thrust setpoint)
        self._throttle = 0.38

        ## Initialising thrust value from ceiling effect
        self.thrustCE = 0.0

    """ Functions for calculating open left-right fuzzification for Membership Functions (MF) """
    def openLeft(self, curr_value, alpha, beta):
        ## x is the current value, alpha and beta are the extreme left and right values of the range respectively.
        if curr_value < alpha:
            return 1
        elif alpha < curr_value and curr_value <= beta:
            return (beta - curr_value) / (beta - alpha)
        else:
            return 0

    def openRight(self, curr_value, alpha, beta):
        if curr_value < alpha:
            return 0
        elif alpha < curr_value and curr_value <= beta:
            return (curr_value - alpha) / (beta - alpha)
        else:
            return 0

    """ Functions for calculating triangular fuzzification"""
    def triangular(self, curr_value, left, center, right):
        return max(min((curr_value-left)/(center-left), (right-curr_value)/(right-center)), 0)

    def halftriangular(self, curr_value, left, right):
        return ((right-curr_value)/(right-left))
    
    """ Fuzzy Membership Functions for Descriptors """
    def partitionCD(self, curr_value):
        NL = 0; NS = 0; Z = 0; PS = 0; PL = 0
        """ Need to check on this triangle values. Might need to reduce granularity """  ## <------------ TO CHECK
        if curr_value > 0 and curr_value < 0.2:
            NL = self.halftriangular(curr_value, 0.0, 0.2)
        # elif curr_value > 0.1 and curr_value < 0.3:
        #     NM = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > 0.0 and curr_value < 0.4:
            NS = self.triangular(curr_value, 0.0, 0.2, 0.4)
        if curr_value > 0.2 and curr_value < 0.6:
            Z = self.triangular(curr_value, 0.2, 0.4, 0.6)
        if curr_value > 0.4 and curr_value < 0.8:
            PS = self.triangular(curr_value, 0.4, 0.6, 0.8)
        # elif curr_value > 0.5 and curr_value < 0.7:
        #     PM = self.triangular(curr_value, 0.5, 0.6, 0.7)
        if curr_value > 0.8 and curr_value < 1.0:
            PL = self.openRight(curr_value, 0.6, 0.8)

        return NL, NS, Z, PS, PL

    def partitionSD(self, curr_value):
        NL = 0; NS = 0; Z = 0; PS = 0; PL = 0
        if curr_value > 0 and curr_value < 0.2:
            NL = self.halftriangular(curr_value, 0.0, 0.2)
        # elif curr_value > 0.1 and curr_value < 0.3:
        #     NM = self.triangular(curr_value, 0.1, 0.2, 0.3)
        if curr_value > 0.0 and curr_value < 0.4:
            NS = self.triangular(curr_value, 0.0, 0.2, 0.4)
        if curr_value > 0.2 and curr_value < 0.6:
            Z = self.triangular(curr_value, 0.2, 0.4, 0.6)
        if curr_value > 0.4 and curr_value < 0.8:
            PS = self.triangular(curr_value, 0.4, 0.6, 0.8)
        # elif curr_value > 0.5 and curr_value < 0.7:
        #     PM = self.triangular(curr_value, 0.5, 0.6, 0.7)
        if curr_value > 0.8 and curr_value < 1.0:
            PL = self.openRight(curr_value, 0.6, 0.8)

        return NL, NS, Z, PS, PL

    """ Rules implementation """
    def compare(self, TC1, TC2, TC3, TC4, TC5, TC6, TC7, TC8):
        TC = 0.0

        ## New method for comparison for more than 2 items:
        check_list = np.array([TC1, TC2, TC3, TC4, TC5, TC6, TC7, TC8])
        if not not check_list.any():
            TC = min(check_list[check_list != 0]) ## Take min item that is not zero.
        else:
            TC = 0.0 ## If array contains only zero element items, TC = 0
        return TC


    """ 
    Input rules here 
    With reference to the FLC rule table, only consider those that returns PL
    Very Near is NL, Near is NS, Norm is Z, Far is PS, Very Far is PL
    """
    def rules(self):
        ## Rules for PL
        PLTC1 = min(self.PLCD, self.NLSD)
        PLTC2 = min(self.PLCD, self.NSSD)
        PLTC3 = min(self.PLCD, self.ZSD)
        PLTC4 = min(self.PSCD, self.NLSD)
        PLTC5 = min(self.PSCD, self.NSSD)
        PLTC6 = min(self.ZCD, self.NLSD)
        PLTC7 = 0.0
        PLTC8 = 0.0
        self.PLTC = self.compare(PLTC1, PLTC2, PLTC3, PLTC4, PLTC5, PLTC6, PLTC7, PLTC8)

        ## Rules for PS
        PSTC1 = min(self.PLCD, self.PSSD)
        PSTC2 = min(self.PLCD, self.PLSD)
        PSTC3 = min(self.PSCD, self.ZSD)
        PSTC4 = min(self.PSCD, self.PSSD)
        PSTC5 = min(self.PSCD, self.PLSD)
        PSTC6 = min(self.ZCD, self.NSSD)
        PSTC7 = min(self.NSCD, self.NLSD)
        PSTC8 = min(self.NSCD, self.NSSD)
        self.PSTC = self.compare(PSTC1, PSTC2, PSTC3, PSTC4, PSTC5, PSTC6, PSTC7, PSTC8)
        
        ## Rules for Z
        ZTC1 = min(self.NLCD, self.NLSD)
        ZTC2 = min(self.NLCD, self.NSSD)
        ZTC3 = min(self.NLCD, self.ZSD)
        ZTC4 = min(self.ZCD, self.ZSD)
        ZTC5 = min(self.NSCD, self.ZSD)
        ZTC6 = 0.0
        ZTC7 = 0.0
        ZTC8 = 0.0
        self.ZTC = self.compare(ZTC1, ZTC2, ZTC3, ZTC4, ZTC5, ZTC6, ZTC7, ZTC8)

        ## Rules for NS
        NSTC1 = min(self.ZCD, self.PSSD)
        NSTC2 = min(self.ZCD, self.ZSD)
        NSTC3 = min(self.NSCD, self.PSSD)
        NSTC4 = 0.0
        NSTC5 = 0.0
        NSTC6 = 0.0
        NSTC7 = 0.0
        NSTC8 = 0.0
        self.NSTC = self.compare(NSTC1, NSTC2, NSTC3, NSTC4, NSTC5, NSTC6, NSTC7, NSTC8)

        ## Rules for NL
        NLTC1 = min(self.ZCD, self.PLSD)
        NLTC2 = min(self.NSCD, self.PSSD)
        NLTC3 = min(self.NSCD, self.PLSD)
        NLTC4 = min(self.NLCD, self.ZSD)
        NLTC5 = min(self.NLCD, self.PSSD)
        NLTC6 = min(self.NLCD, self.PLSD)
        NLTC7 = 0.0
        NLTC8 = 0.0
        self.NLTC = self.compare(NLTC1, NLTC2, NLTC3, NLTC4, NLTC5, NLTC6, NLTC7, NLTC8)


    """ Defuzzification """
    def areaTR(self, mu, left, center, right):
        x1 = mu*(center-left)+left
        x2 = right-mu*(right-center)
        d1 = (right-left)  # getting the base length
        d2 = x2-x1
        area = 0.5*mu*(d1+d2)  # mu here is the height of the region of interest (roi)
        return area

    def areahalfTR(self, mu, left, right):
        area = 0.5*mu*(right-left)
        return area 

    def areaOL(self, mu, left, right):
        extremeL = 0
        # xOL = right*mu(right-left)
        aOL = (0.5*mu*(right-left))+((left-extremeL)*mu)
        return aOL, (right-extremeL)/2

    def areaOR(self, mu, left, right):
        extremeR = 0.6
        # xOR = (right-left)*mu+left  # xOR is x OpenRight
        # aOR = 0.5*mu*((0.6-left)+(0.6-xOR))  # 0.6 here is the max, aOR is area OpenRight
        aOR = (0.5*mu*(right-left))+(mu*(extremeR-right))
        print(mu, left, right)
        return aOR, (extremeR-left)/2 + left
    
    def defuzzification(self):
        areaNL = 0
        areaNS = 0
        areaZ = 0
        areaPS = 0
        areaPL = 0
        
        cNL = 0
        cNS = 0
        cZ = 0
        cPS = 0
        cPL = 0

        """ 
        To Check:
          Probably need to update this table variables
          UAV thrust cannot be lesser than 0.35 when near ceiling,
          UAV thrust cannot be more than x value at any point, might damage prismatic joints
        """
        if self.NLTC != 0:
            areaNL, cNL = self.areaOL(self.NLTC, 0.25, 0.35)
        
        if self.NSTC != 0:
            areaNS = self.areaTR(self.NSTC, 0.3, 0.35, 0.4)
            cNS = 0.35
        
        if self.ZTC != 0:
            areaZ = self.areaTR(self.ZTC, 0.35, 0.4, 0.45)
            cZ = 0.4
        
        if self.PSTC != 0:
            areaPS = self.areaTR(self.PSTC, 0.4, 0.45, 0.5)
            cPS = 0.45
        
        if self.PLTC != 0:
            areaPL, cPL = self.areaOR(self.PLTC, 0.45, 0.55)

        numerator = (areaNL*cNL) + (areaNS*cNS) + (areaZ*cZ) + (areaPS*cPS) + (areaPL*cPL)
        # print("num: ", numerator)
        denominator = areaNL + areaNS + areaZ + areaPS + areaPL
        # print("denom: ", denominator)
        if denominator == 0:
            print("No rules exist to give the results")
            print("Default thrust setpoint at: ", self._throttle)
            return (self._throttle)
        else:
            crispOp = np.round(numerator,5)/np.round(denominator,5)
            return (crispOp)


    def update(self):
        """ Update thrustCE """
        runCE = thrustCE(self._ceiling_dist)
        self.thrustCE = runCE.getThrust()
        print("Thrust from CE (N): ", self.thrustCE)

        """ Update all fuzzy values for all inputs of the fuzzy sets """
        self.NLCD, self.NSCD, self.ZCD, self.PSCD, self.PLCD = self.partitionCD(self._ceiling_dist)
        self.NLSD, self.NSSD, self.ZSD, self.PSSD, self.PLSD = self.partitionSD(self._vel_uav)
        OP = [[self.NLCD, self.NSCD, self.ZCD, self.PSCD, self.PLCD],
              [self.NLSD, self.NSSD, self.ZSD, self.PSSD, self.PLSD]]
        print("The fuzzy values of the crisp inputs are: ", np.round(OP, 4))

        self.rules()
        OpRules = [[self.NLTC, self.NSTC, self.ZTC, self.PSTC, self.PLTC]]
        print("Output Rules: ", np.round(OpRules, 4))

        crispOpFinal = self.defuzzification()
        print("\n The crisp TC value is: ", crispOpFinal)


def main():
    dist = float(input("What is the distance? "))
    spd = float(input("What is the speed of the UAV? "))
    print("\n dist: ", dist)
    print("\n speed: ", spd)
    run = flc(dist, spd)
    run.update()


if __name__ == "__main__":
    main()