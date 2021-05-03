1.特别注意字符集的设置，因为本项目中使用到WIN32下的串口，所以需要使用到WIN32的API，项目属性配置中需要配置字符集为“使用Unicode字符集”，否则编译会报错。

2.WIN32下FreeRTOS任务运行间隔时间不是很稳定，主要是由于本来给FreeRTOS的线程频率就不稳定，也就是说FreeRTOS的sysTick与真实的时间不同步。为了避免计算的时候不准确，所以统一都用FreeRTOS的sysTick就好。

