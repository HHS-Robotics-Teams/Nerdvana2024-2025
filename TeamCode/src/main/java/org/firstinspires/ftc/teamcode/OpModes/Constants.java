package org.firstinspires.ftc.teamcode.OpModes;

public class Constants {
    //pivot motor values
    public static final int pivotmstartpos = 0;
    public static final int pivotmdrivepos =60;
    public static final int pivotmpickuppos = 340;
    public static final int pivotmlowbucket = 1750;
    public static final int pivotmhighbucket = 2300;
    public static final int pivotmlowchamber = 870;
    public static final int pivotmhighchamber = 1640;

    public static final int pivotmclimbpos = 3300;

    //extendo positions
    public static final int EXTENDOINPOS = 1;
    public static final int EXTENDOOUTPOS = 0;

    //Rodo-Intake positions
    public static final int intakeleftpos = 0;
    public static final double intakecenterpos = .5;
    public static final int intakerightpos = 1;



    //Flags
    public static boolean initPositionsReached = false;
    public static boolean rodoControlReached = false;
    public static boolean EXTENDOMINREACHED = false;
    public static boolean EXTENDOMAXREACHED = false;
    public static boolean climbPositionReached = false;
}
