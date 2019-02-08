package frc.robot.driver;

import frc.robot.common.robotprovider.*;
import frc.robot.driver.common.IControlTask;
import frc.robot.driver.controltasks.*;
import frc.robot.ElectronicsConstants;

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
        boolean switchA = !this.dipSwitchA.get();
        boolean switchB = !this.dipSwitchB.get();
        boolean switchC = !this.dipSwitchC.get();
        boolean switchD = !this.dipSwitchD.get();

        boolean startCenter = switchA != switchB;
        boolean startLeft = switchA && switchB;
        boolean startRight = !switchA && !switchB;
        boolean startSlot = switchC; // if center -> left(t)/right(f), if side -> front(t)/side(f)
        boolean placeTwo = switchD;

        // print routine parameters to the smartdash
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "startCenter", startCenter);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "startLeft", startLeft);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "startRight", startRight);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "startSlot", startSlot);
        this.logger.logBoolean(AutonomousRoutineSelector.LogName, "placeTwo", placeTwo);

        if(startCenter)
        {
            if(startSlot)
            {
                if(placeTwo)
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
                if(placeTwo)
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
            if(startSlot)
            {
                if(placeTwo)
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
                if(placeTwo)
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
            if(startSlot)
            {
                if(placeTwo)
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
                if(placeTwo)
                {
                    return startRightGoSidePlace2();
                }
                else
                {
                    return startRightGoSidePlace1();
                }
            }
        }

        return AutonomousRoutineSelector.GetFillerRoutine();
    }

    /**
     * Gets an autonomous routine that does nothing
     * 
     * @return very long WaitTask
     */
    private static IControlTask GetFillerRoutine()
    {
        return new DriveVelocityTimedTask(16.0, 1.0, 0.0);
    }

    private static IControlTask startCenterGoLeftPlace2()
    {
        return;
    }

    private static IControlTask startCenterGoLeftPlace1()
    {
        return;
    }
    private static IControlTask startCenterGoRightPlace2()
    {
        return;
    }
    private static IControlTask startCenterGoRightPlace1()
    {
        return;
    }
    private static IControlTask startLeftGoFrontPlace2()
    {
        return;
    }

    private static IControlTask startLeftGoFrontPlace1()
    {
        return;
    }

    private static IControlTask startLeftGoSidePlace2()
    {
        return;
    }

    private static IControlTask startLeftGoSidePlace1()
    {
        return;
    }

    private static IControlTask startRightGoFrontPlace2()
    {
        return;
    }

    private static IControlTask startRightGoFrontPlace1()
    {
        return;
    }

    private static IControlTask startRightGoSidePlace2()
    {
        return;
    }

    private static IControlTask startRightGoSidePlace1()
    {
        return;
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
