import React from 'react'

export default function TaskHistory({ tasks, title, showAll = false }) {
  if (!tasks || tasks.length === 0) {
    return (
      <div className="glass-card p-8 text-center text-gray-500">
        <div className="text-4xl mb-3">📭</div>
        <p>暂无任务历史</p>
      </div>
    )
  }

  const getStatusIcon = (status) => {
    switch (status) {
      case 'completed': return '✅'
      case 'processing': return '🔄'
      case 'partial': return '⚠️'
      case 'failed': return '❌'
      default: return '📝'
    }
  }

  const getStatusColor = (status) => {
    switch (status) {
      case 'completed': return 'text-green-400'
      case 'processing': return 'text-blue-400'
      case 'partial': return 'text-yellow-400'
      case 'failed': return 'text-red-400'
      default: return 'text-gray-400'
    }
  }

  const displayTasks = showAll ? tasks : tasks.slice(-3).reverse()

  return (
    <div className="glass-card p-6 animate-slide-in">
      <div className="flex items-center justify-between mb-4">
        <h2 className="text-xl font-semibold flex items-center gap-2">
          <span className="text-2xl">📋</span>
          {title}
        </h2>
        <span className="text-sm text-gray-400">{tasks.length} 个任务</span>
      </div>

      <div className="space-y-3">
        {displayTasks.map((task, index) => (
          <div
            key={task.id || index}
            className="p-4 bg-white/5 border border-white/10 rounded-xl hover:bg-white/10 transition-all duration-200"
          >
            <div className="flex items-start justify-between mb-3">
              <div className="flex-1">
                <div className="flex items-center gap-2 mb-1">
                  <span className={`text-lg ${getStatusColor(task.status)}`}>
                    {getStatusIcon(task.status)}
                  </span>
                  <span className="text-xs text-gray-500 font-mono">#{task.id || index + 1}</span>
                  <span className="text-xs text-gray-500">•</span>
                  <span className="text-xs text-gray-500">{task.timestamp}</span>
                </div>
                <p className="text-white font-medium">{task.input}</p>
              </div>
            </div>

            {/* 步骤列表 */}
            {task.steps && task.steps.length > 0 && (
              <div className="mt-3 space-y-2">
                <div className="text-xs text-gray-500 mb-2">执行步骤：</div>
                {task.steps.map((step, stepIndex) => (
                  <div
                    key={stepIndex}
                    className="flex items-center gap-2 text-sm pl-3 border-l-2 border-primary-500/30"
                  >
                    <span className="text-primary-400">▸</span>
                    <code className="text-primary-300 font-mono">{step.tool}</code>
                    <span className="text-gray-500 text-xs">
                      ({Object.entries(step.args || {}).map(([k, v]) => `${k}=${v}`).join(', ')})
                    </span>
                  </div>
                ))}
              </div>
            )}

            {/* 总结 */}
            {task.summary && (
              <div className="mt-3 pt-3 border-t border-white/10">
                <span className="text-sm text-gray-400">{task.summary}</span>
              </div>
            )}

            {/* 错误信息 */}
            {task.error && (
              <div className="mt-3 p-2 bg-red-500/10 border border-red-500/30 rounded-lg">
                <span className="text-sm text-red-400">⚠️ {task.error}</span>
              </div>
            )}
          </div>
        ))}
      </div>
    </div>
  )
}
