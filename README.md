# hscFsm_bridge

华数状态机的开发文档

主要的开发步骤

继承 HsTaskFramework 类的接口  如 抓起功能的 PickPlaceTask 任务状态机

    class PickPlaceTask :public HsTaskFramework
    {
    public:
        PickPlaceTask();

        /**
         * @brief init 初始化
         */
        virtual void init();

        /**
         * @brief quit 任务强制退出
         */
        virtual void quit();

        /**
         * @brief registerTaskList 任务状态机的注册
         * @return
         */
        virtual bool registerTaskList();


	注册的内容有表达形式：
    /**
     * @brief HsFsm::HsTaskFramework::registerTask
     * @param state 当前的状态
     * @param behevior 当前状态所属的行为
     * @param call 状态-行为 所属的函数行为
     */
	registerTask("init","initing", [&](callParm  &parm));
    

如 PickPlaceTask的状态机的部分步骤

---
使用的步骤：
 1. 启动hscfsm_bridge     -- rosrun hscfsm_bridge hscfsm_bridge
 2. 在路径下找到Task的动态库  rosservice call /getTaskList "{}"
 3. 设置task 的任务           rosservice call /setTaskServer "mode: false
	主要的参数有：
		mode: false //设置的模式 直接加载为调试模式 ，动态库加载为部署模式
		taskId: 1 //为直接加载的模式使用
		taskName: 'PickPlaceTask'" //加载的任务名称 需要与动态库的名称保持一致

4. 启动 rosservice call /startTaskServer //开始task服务
5. 关闭 rosservice call /stopTaskServer
