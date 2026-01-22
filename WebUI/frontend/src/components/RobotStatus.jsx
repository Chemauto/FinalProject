import React from 'react'

export default function RobotStatus({ status }) {
  const getStatusInfo = () => {
    if (!status) {
      return {
        icon: '⚠️',
        title: '未连接',
        description: '正在连接到机器人服务...',
        color: 'yellow'
      }
    }

    if (status.is_processing) {
      return {
        icon: '🔄',
        title: '执行中',
        description: '机器人正在执行任务',
        color: 'blue'
      }
    }

    return {
      icon: '✅',
      title: '就绪',
      description: '机器人系统运行正常',
      color: 'green'
    }
  }

  const statusInfo = getStatusInfo()
  const colorClasses = {
    green: 'from-green-500/20 to-emerald-500/20 border-green-500/30',
    blue: 'from-blue-500/20 to-cyan-500/20 border-blue-500/30',
    yellow: 'from-yellow-500/20 to-orange-500/20 border-yellow-500/30',
  }

  return (
    <div className="glass-card p-6 animate-slide-in">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-4">
          <div className={`w-16 h-16 rounded-2xl bg-gradient-to-br ${colorClasses[statusInfo.color]} border flex items-center justify-center text-3xl`}>
            {statusInfo.icon}
          </div>
          <div>
            <h3 className="text-lg font-semibold">{statusInfo.title}</h3>
            <p className="text-gray-400 text-sm">{statusInfo.description}</p>
          </div>
        </div>

        {status && (
          <div className="grid grid-cols-3 gap-6">
            <div className="text-center">
              <div className="text-2xl font-bold text-primary-400">{status.total_tasks || 0}</div>
              <div className="text-xs text-gray-400 mt-1">总任务数</div>
            </div>
            <div className="text-center">
              <div className="text-2xl font-bold text-green-400">{status.available_tools || 0}</div>
              <div className="text-xs text-gray-400 mt-1">可用工具</div>
            </div>
            <div className="text-center">
              <div className={`text-2xl font-bold ${status.api_configured ? 'text-green-400' : 'text-red-400'}`}>
                {status.api_configured ? '✓' : '✗'}
              </div>
              <div className="text-xs text-gray-400 mt-1">API 状态</div>
            </div>
          </div>
        )}
      </div>
    </div>
  )
}
