package frc.robot.driver;

import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.controltasks.*;
import frc.robot.ElectronicsConstants;
import frc.robot.TuningConstants;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class AutonomousRoutineSelector
{
    private static final String LogName = "auto";
    private final IDashboardLogger logger;
    private final IDriverStation driverStation;

    private final IDigitalInput dipSwitchA;
    private final IDigitalInput dipSwitchB;
    private final IDigitalInput dipSwitchC;
    private final IDigitalInput dipSwitchD;

    /**
     * Initializes a new AutonomousRoutineSelector
     */
    @Inject
    public AutonomousRoutineSelector(
        IDashboardLogger logger,
        IRobotProvider provider)
    {
        // initialize robot parts that are used to select autonomous routine (e.g. dipswitches) here...
        this.logger = logger;
        this.dipSwitchA = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_A_DIGITAL_CHANNEL);
        this.dipSwitchB = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_B_DIGITAL_CHANNEL);
        this.dipSwitchC = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_C_DIGITAL_CHANNEL);
        this.dipSwitchD = provider.getDigitalInput(ElectronicsConstants.AUTO_DIP_SWITCH_D_DIGITAL_CHANNEL);

        this.driverStation = provider.getDriverStation();
    }

    /**
     * Check what routine we want to use and return it
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine()
    {
        boolean switchA = this.dipSwitchA.get();
        boolean switchB = this.dipSwitchB.get();
        boolean switchC = this.dipSwitchC.get();
        boolean switchD = this.dipSwitchD.get();

        // print routine parameters to the smartdash
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "switchA", switchA);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "switchB", switchB);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "switchC", switchC);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "switchD", switchD);

        boolean startCenter = !switchA && !switchB;
        boolean startLeft = switchA;
        boolean startRight = switchB;
        boolean startSlot = switchC; // if center -> left(t)/right(f), if side -> front(t)/side(f)
        boolean placeTwo = switchD;

        /*
        if (startCenter)
        {
            if (startSlot)
            {
                if (placeTwo)
                {
                    return startCenterGoLeftPlace2();
                }
                else
                {
                    return startCenterGoLeftPlace1();
                }
            }
            else
            {
                if (placeTwo)
                {
                    return startCenterGoRightPlace2();
                }
                else
                {
                    return startCenterGoRightPlace1();
                }
            }
        }
        else if (startLeft)
        {
            if (startSlot)
            {
                if (placeTwo)
                {
                    return startLeftGoFrontPlace2();
                }
                else
                {
                    return startLeftGoFrontPlace1();
                }
            }
            else
            {
                if (placeTwo)
                {
                    return startLeftGoSidePlace2();
                }
                else
                {
                    return startLeftGoSidePlace1();
                }
            }
        }
        else if (startRight)
        {
            if (startSlot)
            {
                if (placeTwo)
                {
                    return startRightGoFrontPlace2();
                }
                else
                {
                    return startRightGoFrontPlace1();
                }
            }
            else
            {
                if (placeTwo)
                {
                    return startRightGoSidePlace2();
                }
                else
                {
                    return startRightGoSidePlace1();
                }
            }
        }*/

        return new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition);
    }

    /**
     * Gets an autonomous routine that does nothing
     * 
     * @return very long WaitTask
     */
    private static IControlTask GetFillerRoutine()
    {
        return new WaitTask(0);
    }

    private static IControlTask startCenterGoLeftPlace2()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("CenterL1 to FrontL.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(-180), // change direction depending on whether we are more likely to over or under shoot
            new FollowPathTask("FrontL to LLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(-180),
            new FollowPathTask("LLoading to L1.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startCenterGoLeftPlace1()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("CenterL1 to FrontL.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }
    private static IControlTask startCenterGoRightPlace2()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("CenterL1 to FrontR.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("FrontR to RLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("RLoading to R1.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );

    }
    private static IControlTask startCenterGoRightPlace1()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("CenterL1 to FrontR.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }
    private static IControlTask startLeftGoFrontPlace2()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("LeftL1 to FrontL.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("FrontL to LLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("LLoading to L1.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startLeftGoFrontPlace1()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("LeftL1 to FrontL.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startLeftGoSidePlace2()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("LeftL1 to L1.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("L1 to LLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("LLoading to L2.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startLeftGoSidePlace1()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("LeftL1 to L1.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startRightGoFrontPlace2()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("RightL1 to FrontR.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("FrontR to RLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("RLoading to R1.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startRightGoFrontPlace1()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("RightL1 to FrontR.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startRightGoSidePlace2()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("RightL1 to R1.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("R1 to RLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("RLoading to R2.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

    private static IControlTask startRightGoSidePlace1()
    {
        return SequentialTask.Sequence(
            ConcurrentTask.AllTasks(
                new GrabberSetWristPositionTask(Operation.GrabberWristHatchPosition),
                new FollowPathTask("RightL1 to R1.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5)
        );
    }

}








































































































































/*
                                      .                                                             
                                    .;+;+                                                           
                                    .+;;'   `,+'.                                                   
                                    ;';;+:..`` :+'+                                                 
                                    ,'+`    .+;;;;;+                                                
                                     ;,,, .+;;;;;'+++;                                              
                                     ;' `+;;;;;#+'+'+''#:.                                          
                                     '`+';;;'+;+;+++'''+'.                                          
                                     #';;;;#';+'+'''+''+'                                           
                                     ;;;;#;,+;;+;;;'''''':                                          
                                     ';'++'.`+;;'';;''+'',                                          
                                     :#'#+'``.'+++'#++'':`                                          
                                      `';++##```##+.''.##                                           
                                      +++#   #`#  `++++                                             
                                      +'#+ # :#: # ##'+                                             
                                      `#+#   +`+   #'#`                                             
                                       :,.+,+,`:+,+..,                                              
                                       `,:```,`,`.`;,                                               
                                        :+.;``.``;.#;                                               
                                        .'``'+'+'``'.                                               
                                         ,````````..                                                
                                          :```````:                                                 
                                          +``.:,``'                                                 
                                          :```````:                                                 
                                           +`````+                                                  
                                            ';+##                                                   
                                            '```'                                                   
                                           `'```'`                                                  
                                         .+''''''''                                                 
                                        +;;;;;;;;''#                                                
                                       :       `   `:                                               
                                      `,            '                                               
                                      +              '                                              
                                     ,;';,``.``.,,,:;#                                              
                                     +;;;;;;;;;;;;;;;'                                              
                                    ,';;;;;;;;;;;;;;;',                                             
                                    +:;;;;;;';;;;;;;;;+                                             
                                   `.   .:,;+;;:::;.``,                                             
                                   :`       #,       `.`                                            
                                   +       # ;        .;                                            
                                  .;;,`    ,         `,+                                            
                                  +;;;;;;''';;;;;;;';;';                                            
                                  +;;;;;;;';;;;;;;;;;'';;                                           
                                 `';;;;;;';;;;;;;;;;;';;+                                           
                                 + `:;;;;+;;;;;;;;';'''::                                           
                                 '     `:  ```````    ,  ,                                          
                                :       '             ;  +                                          
                                '`     ..             ,  ,                                          
                               ,;;;;;..+,`        ```.':;',                                         
                               +;;;;;;'+;;;;;;;;;;;;;;+;;;+                                         
                               ';;;;;;++;;;;;;;;;;;;;;';;;+                                         
                              `.:';;;;;#;;;;;;;;;;;;;;';;;;`                                        
                              ;    `,; ',:;;';;';;;;;:;``  +                                        
                              +      ; ;              ;    `                                        
                              ;      : +              '    `;                                       
                              ';:`` `` '              :`,:;;+                                       
                             `';;;;'+  +,..```````..:;#;;;;;;.                                      
                             `;;;;;;+  +;;;;;;;;;;;;;':';;;;;#                                      
                             .;;;;;;+  ';;;;;;;;;;;;;;,';;;;` .                                     
                             : `.;;'+  +;;;;;;;;;;;;;','.`    +                                     
                             '      ;  +.,,;:;:;;;,..`: ,     ``                                    
                             +      ,  '              : ;   .;'+                                    
                             +.`   ``  +              ;  ;:;;;;':                                   
                             ';;;';;`  +             .'  ;;;;;;;+                                   
                             ';;;;;'   :+++#++##+#+''',   +;;;;.`.                                  
                             +;;;;;'   +;;::;;;+:+;;'',   ,;;.   +                                  
                            ``:;;;;+   +;;:;;;:+;+;;++;    +     .`                                 
                             `   ``'   +;;;;;;;+;+;;'+;     ,   ;#,                                 
                            .      ;   ';;;;;;;;;;;;++'     + .+``.;                                
                            ``     ;   ';;;;;;+;';;;'+'      #`````:,                               
                             +++;,:.   ':;''++;:';:;'';      +``````,`                              
                             ,```,+    +;;';:;;+;;;;'';      +``````,+                              
                            .``````:   ;:;;++';;;;;;';,      ,``:#``+`.                             
                            ,``````'   `';;;;:;;;;;;+;`     '+``+:'`..'                             
                            ,``````'    +;;;;;;;;;;;''     ;:'``#;;.`++                             
                            ```````;    `;:;;;;;;;;;;#     ':'``++:+`+;                             
                            ```'`.`;     +;;;;;;;;;;;+    :::#``' +#`';                             
                            ,``'`:`#     `';;;;;;;;;;+    +:'.`,. ++`;;                             
                            +`.``+`'     :#;;;;;;;;;;;`   +:# ,`  +;`.'                             
                           ,.`+`.:.      ##;;;;;;;;;;;'   ,'`     ;:+#                              
                           '`;.`+`#      ##+;;;;;;;;;;+          ,::;                               
                           ,+,`:``,     :###;;;;;;;;;:'          +:;`                               
                            '`,,`+      ';##';;;;;;;;;;.         +:#                                
                             '+.+       +;;##;;;;;;;;;;'         ;:;                                
                               `       :;;;+#;;;;;;;;;;+        ;::`                                
                                       +;;;;#+;;;;;;;;;;        +:'                                 
                                       ';;;;+#;;;;;;;;;;.       ;:'                                 
                                      ,;;;;;;#;;;;;;;;;;+      +::.                                 
                                      +;;;;;;'';;;;;;;;;'      +:+                                  
                                     `;;;;;;;;#;;;;;;;;;;`    `;:+                                  
                                     ,;;;;;;;;+;;;;;;;;;;+    ':;,                                  
                                     +;;;;;;;;;+;;;;;;;;;'    +:+                                   
                                    .;;;;;;;;;+,;;;;;;;;;;`   ;;+                                   
                                    ';;;;;;;;;, ';;;;;;:;;,  +;:,                                   
                                    ';;;;;;;;'  +;;;;;;;;;'  +:+                                    
                                   ;;;;;;;;;;+  ,;;;;;;;;;+  ;:'                                    
                                   +;;;;;;;;;    ';;;;;;;;;`;:;`                                    
                                   ;;;;;;;;;+    +;;;;;;;;;+#:+                                     
                                  ';;;;;;;;;:    ;;;;;;;;;;';:'                                     
                                 `';;;;;;;:'      ';;;;;;;;;;:.                                     
                                 .;;;;;;;;;+      +;;;;;;;;;'+                                      
                                 +;;;;;;;;;       ';;;;;;;;;#+                                      
                                `;;;;;;;;;+       `;;;;;;;;;;`                                      
                                +;;;;;;;;;.        +;;;;;;;;;`                                      
                                ';;;;;;;:'         ;;;;;;;;;;;                                      
                               :;;;;;;;;;:         `;;;;;;;;;+                                      
                               +;;;;;;;;;           ';;;;;;;;;`                                     
                               ;;;;;;;;;+           ';;;;;;;;;:                                     
                              ';;;;;;;;;;           ,;;;;;;;;;+                                     
                              ':;;;;;;;'             +;;;;;;;;;                                     
                             .;:;;;;;;;'             +;;;;;;;;;:                                    
                             +;;;;;;;;;`             .;;;;;;;;;+                                    
                            `;;;;;;;;;+               ;:;;;;;;;;`                                   
                            ;;;;;;;;;;.               +;;;;;;;::.                                   
                            ';;;;;;;;'`               :;;;;;;;;:+                                   
                           :;;;;;;;;:'                ';;;;;;;;;'                                   
                           ';;;;;;;;'`                +#;;;;;;;;;`                                  
                          `;;;;;;;;;+                 '';;;;;;;;;+                                  
                          +;;;;;;;;;.                '::;;;;;;;;;+                                  
                          ;;;;;;;;;+                 #:'';;;;;;;;;`                                 
                         .#;;;;;;;;'                `;:+;;;;;;;;;;;                                 
                         ':'';;;;;;                 '::.,;;;;;;;;;+                                 
                        +::::+';;;+                 ':'  +:;;;;;;;;`                                
                       `;;;::::;#+:                `;:+  +;;;;;;;:;;      '#+,                      
                       +#::::::::;'`               +:;,  `;;;;:;;'#';;;;;::;:'`                     
                       ;:''::::::::#`              +:'    ';:;;+'::;;:;::::::''                     
                       +::;+':::::::'.            .:;+    '''+;::;:;:::;:::;':'                     
                        ';;:;'';:::::':           +::.     +:::::::::::::;#;:#                      
                         .''##;#;:;;:::'+        `+;'      ;:;::::::::;'+;:'+                       
                           ` `+:;+:;::;::+       +:;#      ';:::;:+#+';:::+.                        
                              ,+::+#';::;+       ';::      #:;;'+';'''++:`                          
                                '':::;'''#      ,:;;`      #';:;;:+                                 
                                 `:'++;;':       :++       .;;:;;#,                                 
                                       `                    '':``                                   


*/
