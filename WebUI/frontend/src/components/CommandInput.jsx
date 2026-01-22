import React, { useState, useRef, useEffect } from 'react'

export default function CommandInput({ onSubmit, disabled, isProcessing }) {
  const [input, setInput] = useState('')
  const [suggestions, setSuggestions] = useState([
    '向前移动 1 米',
    '向后移动 0.5 米',
    '向左转 90 度',
    '检测颜色并移动',
    '向前移动 2 米然后向右转 45 度',
  ])
  const textareaRef = useRef(null)

  const handleSubmit = (e) => {
    e?.preventDefault()
    if (input.trim() && !disabled) {
      onSubmit(input.trim())
      setInput('')
    }
  }

  const handleSuggestionClick = (suggestion) => {
    setInput(suggestion)
    textareaRef.current?.focus()
  }

  // 自动调整高度
  useEffect(() => {
    if (textareaRef.current) {
      textareaRef.current.style.height = 'auto'
      textareaRef.current.style.height = textareaRef.current.scrollHeight + 'px'
    }
  }, [input])

  return (
    <div className="glass-card p-6 animate-slide-in">
      <h2 className="text-xl font-semibold mb-4 flex items-center gap-2">
        <span className="text-2xl">💬</span>
        发送命令
      </h2>

      <form onSubmit={handleSubmit} className="space-y-4">
        <div className="relative">
          <textarea
            ref={textareaRef}
            value={input}
            onChange={(e) => setInput(e.target.value)}
            placeholder="输入机器人命令，例如：向前移动 1 米..."
            className="input-field resize-none min-h-[100px] max-h-[200px]"
            disabled={disabled}
            rows={1}
          />

          {/* 字符计数 */}
          <div className="absolute bottom-3 right-3 text-xs text-gray-500">
            {input.length} 字符
          </div>
        </div>

        {/* 建议命令 */}
        <div className="space-y-2">
          <p className="text-sm text-gray-400">💡 快捷命令：</p>
          <div className="flex flex-wrap gap-2">
            {suggestions.map((suggestion, index) => (
              <button
                key={index}
                type="button"
                onClick={() => handleSuggestionClick(suggestion)}
                className="px-4 py-2 bg-white/5 border border-white/10 rounded-lg text-sm text-gray-300 hover:bg-white/10 hover:border-primary-500/50 transition-all duration-200"
                disabled={disabled}
              >
                {suggestion}
              </button>
            ))}
          </div>
        </div>

        {/* 提交按钮 */}
        <div className="flex justify-end">
          <button
            type="submit"
            disabled={disabled || !input.trim()}
            className="btn-primary flex items-center gap-2"
          >
            {isProcessing ? (
              <>
                <svg className="animate-spin h-5 w-5" xmlns="http://www.w3.org/2000/svg" fill="none" viewBox="0 0 24 24">
                  <circle className="opacity-25" cx="12" cy="12" r="10" stroke="currentColor" strokeWidth="4"></circle>
                  <path className="opacity-75" fill="currentColor" d="M4 12a8 8 0 018-8V0C5.373 0 0 5.373 0 12h4zm2 5.291A7.962 7.962 0 014 12H0c0 3.042 1.135 5.824 3 7.938l3-2.647z"></path>
                </svg>
                处理中...
              </>
            ) : (
              <>
                <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                  <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M12 19l9 2-9-18-9 18 9-2zm0 0v-8" />
                </svg>
                发送命令
              </>
            )}
          </button>
        </div>
      </form>
    </div>
  )
}
