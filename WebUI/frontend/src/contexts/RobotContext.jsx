import React, { createContext, useContext, useReducer, useCallback } from 'react'
import axios from 'axios'

const API_BASE = '/api'

// Action types
const ACTIONS = {
  SET_STATUS: 'SET_STATUS',
  SET_TOOLS: 'SET_TOOLS',
  SET_HISTORY: 'SET_HISTORY',
  SET_LOADING: 'SET_LOADING',
  SET_ERROR: 'SET_ERROR',
}

// Initial state
const initialState = {
  status: null,
  tools: [],
  history: [],
  isLoading: false,
  error: null,
  isProcessing: false,
}

// Reducer
function robotReducer(state, action) {
  switch (action.type) {
    case ACTIONS.SET_STATUS:
      return { ...state, status: action.payload, isProcessing: action.payload?.is_processing || false }
    case ACTIONS.SET_TOOLS:
      return { ...state, tools: action.payload }
    case ACTIONS.SET_HISTORY:
      return { ...state, history: action.payload }
    case ACTIONS.SET_LOADING:
      return { ...state, isLoading: action.payload }
    case ACTIONS.SET_ERROR:
      return { ...state, error: action.payload }
    default:
      return state
  }
}

// Create context
const RobotContext = createContext(null)

// Provider component
export function RobotProvider({ children }) {
  const [state, dispatch] = useReducer(robotReducer, initialState)

  const api = axios.create({
    baseURL: API_BASE,
    headers: {
      'Content-Type': 'application/json',
    },
  })

  // Fetch status
  const fetchStatus = useCallback(async () => {
    try {
      const response = await api.get('/status')
      dispatch({ type: ACTIONS.SET_STATUS, payload: response.data })
    } catch (error) {
      console.error('Failed to fetch status:', error)
    }
  }, [])

  // Fetch tools
  const fetchTools = useCallback(async () => {
    try {
      dispatch({ type: ACTIONS.SET_LOADING, payload: true })
      const response = await api.get('/tools')
      dispatch({ type: ACTIONS.SET_TOOLS, payload: response.data })
    } catch (error) {
      dispatch({ type: ACTIONS.SET_ERROR, payload: error.message })
      console.error('Failed to fetch tools:', error)
    } finally {
      dispatch({ type: ACTIONS.SET_LOADING, payload: false })
    }
  }, [])

  // Fetch history
  const fetchHistory = useCallback(async () => {
    try {
      const response = await api.get('/history')
      dispatch({ type: ACTIONS.SET_HISTORY, payload: response.data })
    } catch (error) {
      console.error('Failed to fetch history:', error)
    }
  }, [])

  // Execute command
  const executeCommand = useCallback(async (input) => {
    try {
      const response = await api.post('/command', { input })
      return response.data
    } catch (error) {
      const errorMsg = error.response?.data?.error || error.message
      dispatch({ type: ACTIONS.SET_ERROR, payload: errorMsg })
      throw error
    }
  }, [])

  const value = {
    ...state,
    fetchStatus,
    fetchTools,
    fetchHistory,
    executeCommand,
  }

  return <RobotContext.Provider value={value}>{children}</RobotContext.Provider>
}

// Custom hook
export function useRobot() {
  const context = useContext(RobotContext)
  if (!context) {
    throw new Error('useRobot must be used within a RobotProvider')
  }
  return context
}
