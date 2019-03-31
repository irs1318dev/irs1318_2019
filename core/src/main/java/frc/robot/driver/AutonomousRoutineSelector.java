package frc.robot.driver;

import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.controltasks.*;

import com.google.inject.Inject;
import com.google.inject.Singleton;

@Singleton
public class AutonomousRoutineSelector
{
    private static final String LogName = "auto";
    private final IDashboardLogger logger;
    private final IDriverStation driverStation;

    private final ISendableChooser<StartPosition> positionChooser;
    private final ISendableChooser<AutoRoutine> routineChooser;

    public enum StartPosition
    {
        Left,
        Right,
        Center
    }

    public enum AutoRoutine
    {
        None,
        DeliverTwoRight,
        DeliverTwoLeft,
    }

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

        this.routineChooser = provider.getSendableChooser();
        this.routineChooser.addDefault("None", AutoRoutine.None);
        this.routineChooser.addObject("DeliverTwoRight", AutoRoutine.DeliverTwoRight);
        this.routineChooser.addObject("DeliverTwoLeft", AutoRoutine.DeliverTwoLeft);
        this.logger.addChooser("Auto Routine", this.routineChooser);

        this.positionChooser = provider.getSendableChooser();
        this.positionChooser.addDefault("center", StartPosition.Center);
        this.positionChooser.addObject("left", StartPosition.Left);
        this.positionChooser.addObject("right", StartPosition.Right);
        this.logger.addChooser("Start Position", this.positionChooser);

        this.driverStation = provider.getDriverStation();
    }

    /**
     * Check what routine we want to use and return it
     * @return autonomous routine to execute during autonomous mode
     */
    public IControlTask selectRoutine()
    {
        StartPosition startPosition = this.positionChooser.getSelected();
        AutoRoutine routine = this.routineChooser.getSelected();

        if (routine == AutoRoutine.DeliverTwoLeft)
        {
            switch (startPosition)
            {
                case Center:
                    return startCenterGoLeftPlace2();

                case Left:
                    return startLeftGoSidePlace2();

                case Right:
                    return startLeftGoFrontPlace2();
            }
        }
        else if (routine == AutoRoutine.DeliverTwoRight)
        {
            switch (startPosition)
            {
                case Center:
                    return startCenterGoRightPlace2();

                case Left:
                    return startLeftGoFrontPlace2();

                case Right:
                    return startLeftGoSidePlace2();
            }
        }

        return SequentialTask.Sequence(
            new GrabberSetWristPositionTask(0.5, Operation.GrabberWristHatchPosition),
            new GrabberSetWristPositionTask(0.5, Operation.GrabberWristHatchPosition),
            new GrabberSetWristPositionTask(0.5, Operation.GrabberWristHatchPosition));
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
                new FollowPathTask("/Paths/CenterL1 to FrontL.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(-180), // change direction depending on whether we are more likely to over or under shoot
            new FollowPathTask("/Paths/FrontL to LLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(-180),
            new FollowPathTask("/Paths/LLoading to L1.csv"),
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
                new FollowPathTask("/Paths/CenterL1 to FrontL.csv")),
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
                new DriveDistanceTimedTask(52.0, 1.75)),
            new FollowPathTask("/Paths/CenterL1 to FrontR.csv"),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AllTasks(
                new GrabberKickPanelTask(2.0),
                SequentialTask.Sequence(
                    new WaitTask(0.5),
                    new DriveDistanceTimedTask(-25, 1.5))),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/FrontR to RLoading.csv"),
            ConcurrentTask.AnyTasks(
                new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
                new GrabberPointBeakTask(4.0)),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/RLoading to R1.csv"),
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
                new FollowPathTask("/Paths/CenterL1 to FrontR.csv")),
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
                new FollowPathTask("/Paths/LeftL1 to FrontL.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/FrontL to LLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/LLoading to L1.csv"),
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
                new FollowPathTask("/Paths/LeftL1 to FrontL.csv")),
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
                new FollowPathTask("/Paths/LeftL1 to L1.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/L1 to LLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/LLoading to L2.csv"),
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
                new FollowPathTask("/Paths/LeftL1 to L1.csv")),
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
                new FollowPathTask("/Paths/RightL1 to FrontR.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/FrontR to RLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/RLoading to R1.csv"),
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
                new FollowPathTask("/Paths/RightL1 to FrontR.csv")),
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
                new FollowPathTask("/Paths/RightL1 to R1.csv")),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            ConcurrentTask.AnyTasks(
                new GrabberKickPanelTask(),
                new PIDBrakeTask()),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/R1 to RLoading.csv"),
            new VisionCenteringTask(Operation.VisionEnableCargoShip),
            new VisionAdvanceAndCenterTask(Operation.VisionEnableCargoShip),
            new DriveDistanceTimedTask(-25, 1.5),
            new NavxTurnTask(180),
            new FollowPathTask("/Paths/RLoading to R2.csv"),
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
                new FollowPathTask("/Paths/RightL1 to R1.csv")),
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
