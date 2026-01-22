import { useState, useEffect, useRef } from 'react'

export function useWebSocket(url) {
  const [isConnected, setIsConnected] = useState(false)
  const [lastMessage, setLastMessage] = useState(null)
  const wsRef = useRef(null)
  const reconnectTimeoutRef = useRef(null)
  const reconnectAttempts = useRef(0)
  const MAX_RECONNECT_ATTEMPTS = 5

  useEffect(() => {
    const connect = () => {
      try {
        const ws = new WebSocket(url)
        wsRef.current = ws

        ws.onopen = () => {
          setIsConnected(true)
          reconnectAttempts.current = 0
          console.log('WebSocket connected')
        }

        ws.onclose = () => {
          setIsConnected(false)
          console.log('WebSocket disconnected')

          // Attempt to reconnect
          if (reconnectAttempts.current < MAX_RECONNECT_ATTEMPTS) {
            reconnectAttempts.current++
            const delay = Math.min(1000 * Math.pow(2, reconnectAttempts.current), 30000)
            console.log(`Reconnecting in ${delay}ms... (attempt ${reconnectAttempts.current})`)
            reconnectTimeoutRef.current = setTimeout(connect, delay)
          }
        }

        ws.onerror = (error) => {
          console.error('WebSocket error:', error)
        }

        ws.onmessage = (event) => {
          try {
            const data = JSON.parse(event.data)
            setLastMessage({ data: event.data, parsed: data })
          } catch (error) {
            console.error('Failed to parse WebSocket message:', error)
          }
        }

        // Ping interval
        const pingInterval = setInterval(() => {
          if (ws.readyState === WebSocket.OPEN) {
            ws.send(JSON.stringify({ type: 'ping' }))
          }
        }, 30000)

        return () => {
          clearInterval(pingInterval)
        }
      } catch (error) {
        console.error('Failed to create WebSocket connection:', error)
      }
    }

    connect()

    return () => {
      if (reconnectTimeoutRef.current) {
        clearTimeout(reconnectTimeoutRef.current)
      }
      if (wsRef.current) {
        wsRef.current.close()
      }
    }
  }, [url])

  return { isConnected, lastMessage }
}
