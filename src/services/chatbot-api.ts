import { ChatSession, ChatMessage } from '../types/chatbot';

// TODO: Replace with Hugging Face Space URL after deployment
const API_BASE_URL = 'https://fahadjunaid-rag-chatbot.hf.space';
const TIMEOUT_DURATION = 30000; // 30 seconds

export interface ChatResponse {
  message: ChatMessage;
  sources?: Array<{
    id: string;
    title: string;
    url: string;
    page: string;
    relevance?: number;
  }>;
}

export const startNewSession = async (): Promise<ChatSession> => {
  try {
    const response = await fetch(`${API_BASE_URL}/chat/start`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      signal: AbortSignal.timeout(TIMEOUT_DURATION)
    });

    if (!response.ok) {
      throw new Error(`Failed to start session: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();

    return {
      id: data.session_id || data.id,
      token: data.token || data.session_token,
      createdAt: new Date(),
      isActive: true
    };
  } catch (error) {
    if (error instanceof TypeError && error.message.includes('fetch')) {
      throw new Error('Network error: Please make sure the backend service is running at Hugging Face Space');
    }
    throw error;
  }
};

export const sendMessage = async (token: string, message: string): Promise<ChatResponse> => {
  try {
    const response = await fetch(`${API_BASE_URL}/chat/${token}/message`, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
      },
      body: JSON.stringify({ message }),
      signal: AbortSignal.timeout(TIMEOUT_DURATION)
    });

    if (!response.ok) {
      throw new Error(`Failed to send message: ${response.status} ${response.statusText}`);
    }

    const data = await response.json();

    return {
      message: {
        id: data.id || Date.now().toString(),
        sessionId: data.session_id || token,
        sender: 'bot',
        content: data.response || data.content || '',
        timestamp: new Date(data.timestamp || Date.now()),
        sources: data.sources,
        status: 'delivered'
      },
      sources: data.sources
    };
  } catch (error) {
    if (error instanceof TypeError && error.message.includes('fetch')) {
      throw new Error('Network error: Please make sure the backend service is running at Hugging Face Space');
    }
    throw error;
  }
};