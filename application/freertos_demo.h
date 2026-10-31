// ============================================================================
// freertos_demo.h —— FreeRTOS 任务调度入口（application 层）
// ============================================================================

#pragma once

// 任务调度总入口：创建 start_task 并由其派生全部工作任务。
// 由 User/main.cpp 的 app_main() 在完成初始化后调用（见 freertos_demo.cpp）。
void freertos_demo();
