import React, { useState, useEffect, useRef } from 'react'
import CommandInput from './components/CommandInput'
import TaskHistory from './components/TaskHistory'
import RobotStatus from './components/RobotStatus'
import AvailableTools from './components/AvailableTools'
import { useWebSocket } from './hooks/useWebSocket'
import { RobotProvider, useRobot } from './contexts/RobotContext'

function AppContent() {
  const { status, tools, history, isProcessing, fetchStatus, fetchTools, fetchHistory, executeCommand } = useRobot()
  const { isConnected, lastMessage } = useWebSocket('ws://localhost:5000/ws')
  const [activeTab, setActiveTab] = useState('home')

  // 处理 WebSocket 消息
  useEffect(() => {
    if (lastMessage) {
      const data = JSON.parse(lastMessage.data)
      console.log('WebSocket message:', data)

      // 刷新数据
      if (data.type === 'complete' || data.type === 'progress') {
        fetchStatus()
        fetchHistory()
      }
    }
  }, [lastMessage])

  // 初始加载
  useEffect(() => {
    fetchStatus()
    fetchTools()
    fetchHistory()

    // 定期刷新状态
    const interval = setInterval(() => {
      fetchStatus()
    }, 5000)

    return () => clearInterval(interval)
  }, [])

  const handleCommand = async (input) => {
    await executeCommand(input)
    fetchHistory()
  }

  return (
    <div className="min-h-screen text-white p-4 md:p-8">
      {/* 背景装饰 */}
      <div className="fixed inset-0 overflow-hidden pointer-events-none">
        <div className="absolute top-0 left-1/4 w-96 h-96 bg-primary-500/20 rounded-full blur-3xl animate-pulse-slow" />
        <div className="absolute bottom-0 right-1/4 w-96 h-96 bg-purple-500/20 rounded-full blur-3xl animate-pulse-slow" style={{ animationDelay: '1.5s' }} />
      </div>

      <div className="relative z-10 max-w-7xl mx-auto">
        {/* 头部 */}
        <header className="mb-8">
          <div className="glass-card p-6">
            <div className="flex items-center justify-between">
              <div className="flex items-center gap-4">
                <div className="relative">
                  <div className="w-16 h-16 bg-gradient-to-br from-primary-500 to-primary-600 rounded-2xl flex items-center justify-center shadow-lg shadow-primary-500/30 animate-float">
                    <svg className="w-10 h-10 text-white" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                      <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9.75 17L9 20l-1 1h8l-1-1-.75-3M3 13h18M5 17h14a2 2 0 002-2V5a2 2 0 00-2-2H5a2 2 0 00-2 2v10a2 2 0 002 2z" />
                    </svg>
                  </div>
                  {status?.is_processing && (
                    <div className="absolute -top-1 -right-1 w-4 h-4 bg-green-500 rounded-full border-2 border-slate-900 animate-pulse" />
                  )}
                </div>
                <div>
                  <h1 className="text-3xl font-bold bg-gradient-to-r from-white to-gray-300 bg-clip-text text-transparent">
                    Robot Control
                  </h1>
                  <p className="text-gray-400 mt-1">2D 机器人控制系统</p>
                </div>
              </div>

              <div className="flex items-center gap-3">
                <div className={`flex items-center gap-2 px-4 py-2 rounded-full ${isConnected ? 'bg-green-500/20 text-green-400' : 'bg-red-500/20 text-red-400'}`}>
                  <div className={`w-2 h-2 rounded-full ${isConnected ? 'bg-green-400' : 'bg-red-400 animate-pulse'}`} />
                  <span className="text-sm font-medium">{isConnected ? '已连接' : '未连接'}</span>
                </div>
              </div>
            </div>

            {/* 导航标签 */}
            <div className="flex gap-2 mt-6">
              {[
                { id: 'home', label: '首页', icon: '🏠' },
                { id: 'tools', label: '可用工具', icon: '🔧' },
                { id: 'history', label: '任务历史', icon: '📋' }
              ].map(tab => (
                <button
                  key={tab.id}
                  onClick={() => setActiveTab(tab.id)}
                  className={`px-6 py-3 rounded-xl font-medium transition-all duration-300 ${
                    activeTab === tab.id
                      ? 'bg-primary-500 text-white shadow-lg shadow-primary-500/30'
                      : 'bg-white/5 text-gray-400 hover:bg-white/10'
                  }`}
                >
                  <span className="mr-2">{tab.icon}</span>
                  {tab.label}
                </button>
              ))}
            </div>
          </div>
        </header>

        {/* 主内容区 */}
        <main className="space-y-6">
          {activeTab === 'home' && (
            <>
              {/* 状态卡片 */}
              <RobotStatus status={status} />

              {/* 命令输入 */}
              <CommandInput
                onSubmit={handleCommand}
                disabled={isProcessing}
                isProcessing={isProcessing}
              />

              {/* 最近任务 */}
              {history.length > 0 && (
                <TaskHistory
                  tasks={history.slice(-3)}
                  title="最近任务"
                  showAll={false}
                />
              )}
            </>
          )}

          {activeTab === 'tools' && (
            <AvailableTools tools={tools} />
          )}

          {activeTab === 'history' && (
            <TaskHistory
              tasks={history}
              title="任务历史"
              showAll={true}
            />
          )}
        </main>

        {/* 页脚 */}
        <footer className="mt-12 text-center text-gray-500 text-sm">
          <p>Robot Control Web UI • 基于 MCP 架构的双层 LLM 机器人控制系统</p>
        </footer>
      </div>
    </div>
  )
}

function App() {
  return (
    <RobotProvider>
      <AppContent />
    </RobotProvider>
  )
}

export default App
