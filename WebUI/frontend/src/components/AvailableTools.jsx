import React, { useState } from 'react'

export default function AvailableTools({ tools }) {
  const [selectedTool, setSelectedTool] = useState(null)

  const toolCategories = {
    movement: { name: '移动控制', icon: '🚀', tools: [] },
    vision: { name: '视觉感知', icon: '👁️', tools: [] },
    other: { name: '其他工具', icon: '🔧', tools: [] }
  }

  // 分类工具
  tools?.forEach(tool => {
    if (tool.name.includes('move') || tool.name.includes('turn') || tool.name.includes('stop')) {
      toolCategories.movement.tools.push(tool)
    } else if (tool.name.includes('detect') || tool.name.includes('color')) {
      toolCategories.vision.tools.push(tool)
    } else {
      toolCategories.other.tools.push(tool)
    }
  })

  return (
    <div className="space-y-6">
      <div className="glass-card p-6">
        <h2 className="text-2xl font-bold mb-6 flex items-center gap-3">
          <span className="text-3xl">🔧</span>
          可用工具
        </h2>

        {Object.entries(toolCategories).map(([key, category]) => (
          category.tools.length > 0 && (
            <div key={key} className="mb-6">
              <h3 className="text-lg font-semibold mb-3 flex items-center gap-2">
                <span>{category.icon}</span>
                {category.name}
              </h3>
              <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-3 gap-4">
                {category.tools.map((tool, index) => (
                  <div
                    key={index}
                    onClick={() => setSelectedTool(selectedTool === tool ? null : tool)}
                    className={`p-4 rounded-xl border cursor-pointer transition-all duration-300 ${
                      selectedTool === tool
                        ? 'bg-primary-500/20 border-primary-500 shadow-lg shadow-primary-500/20'
                        : 'bg-white/5 border-white/10 hover:bg-white/10 hover:border-white/20'
                    }`}
                  >
                    <div className="flex items-center gap-2 mb-2">
                      <div className="w-8 h-8 rounded-lg bg-primary-500/30 flex items-center justify-center">
                        <span className="text-sm">⚡</span>
                      </div>
                      <h4 className="font-mono font-semibold text-primary-300">{tool.name}</h4>
                    </div>
                    <p className="text-sm text-gray-400 mb-3">{tool.description}</p>
                    {tool.parameters.length > 0 && (
                      <div className="flex flex-wrap gap-1">
                        {tool.parameters.map((param, i) => (
                          <span
                            key={i}
                            className="px-2 py-1 bg-white/5 rounded text-xs text-gray-500"
                          >
                            {param}
                          </span>
                        ))}
                      </div>
                    )}
                  </div>
                ))}
              </div>
            </div>
          )
        ))}

        {tools?.length === 0 && (
          <div className="text-center py-12 text-gray-500">
            <div className="text-4xl mb-3">📭</div>
            <p>暂无可用工具</p>
          </div>
        )}
      </div>
    </div>
  )
}
