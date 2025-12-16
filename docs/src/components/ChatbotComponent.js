import React, { useState, useRef, useEffect } from 'react';
import { useColorMode } from '@docusaurus/theme-common';

// Custom hook to safely get color mode
function useSafeColorMode() {
  try {
    const { colorMode } = useColorMode();
    return { colorMode };
  } catch (error) {
    // Fallback to system preference or light mode if hook fails
    if (typeof window !== 'undefined') {
      const isDark = window.matchMedia('(prefers-color-scheme: dark)').matches;
      return { colorMode: isDark ? 'dark' : 'light' };
    }
    return { colorMode: 'light' }; // default to light mode during SSR
  }
}

const ChatbotComponent = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [error, setError] = useState(null);
  const messagesEndRef = useRef(null);

  // Use safe color mode hook
  const { colorMode } = useSafeColorMode();

  // Set default color mode if context is not available
  const effectiveColorMode = colorMode || 'light';

  const scrollToBottom = () => {
    if (messagesEndRef.current) {
      messagesEndRef.current.scrollIntoView({ behavior: 'smooth' });
    }
  };

  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const toggleChat = () => {
    setIsOpen(!isOpen);
  };

  const sendMessage = async (e) => {
    e.preventDefault();
    if (!inputValue.trim() || isLoading) return;

    // Use a more reliable unique ID generator
    const generateUniqueId = () => {
      return Date.now() + Math.random();
    };

    const userMessageId = generateUniqueId();
    const userMessage = { id: userMessageId, text: inputValue, sender: 'user' };

    // Update messages with user message first
    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      // Call the backend API to get the response
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: inputValue,
          llm: 'claude', // or 'gemini' depending on preference
          collection: 'rag_embedding'
        }),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data = await response.json();
      const botMessageId = generateUniqueId();
      const botMessage = {
        id: botMessageId,
        text: data.response || 'I received your message but there was an issue processing it.',
        sender: 'bot'
      };

      // Update messages with bot response
      setMessages(prev => {
        // Remove the previous bot message if it exists (to prevent duplicates)
        const filteredMessages = prev.filter(msg => msg.id !== botMessageId || msg.sender !== 'bot');
        return [...filteredMessages, botMessage];
      });
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessageId = generateUniqueId();
      const errorMessage = {
        id: errorMessageId,
        text: 'Sorry, I encountered an error. Please make sure the backend server is running at http://localhost:8000.',
        sender: 'bot'
      };
      setMessages(prev => {
        // Remove the previous error message if it exists
        const filteredMessages = prev.filter(msg => msg.id !== errorMessageId || msg.sender !== 'bot');
        return [...filteredMessages, errorMessage];
      });
    } finally {
      setIsLoading(false);
    }
  };

  const quickQuestions = [
    "What is ROS 2?",
    "How to create a node?",
    "What is QDRant?",
    "Explain AI integration"
  ];

  const handleQuickQuestion = (question) => {
    setInputValue(question);
  };

  return (
    <div className={`chatbot-container ${isOpen ? 'open' : ''}`}>
      {!isOpen ? (
        <button
          className="chatbot-toggle"
          onClick={toggleChat}
          style={{
            background: effectiveColorMode === 'dark' ? '#00d4ff' : '#0088cc',
            color: effectiveColorMode === 'dark' ? '#0a0a12' : 'white',
            border: 'none',
            borderRadius: '50%',
            width: '60px',
            height: '60px',
            fontSize: '24px',
            cursor: 'pointer',
            boxShadow: '0 4px 20px rgba(0, 212, 255, 0.4)',
            position: 'fixed',
            bottom: '30px',
            right: '30px',
            zIndex: '1000',
            display: 'flex',
            alignItems: 'center',
            justifyContent: 'center',
            transition: 'all 0.3s ease'
          }}
          aria-label="Open chatbot"
        >
          💬
        </button>
      ) : (
        <div
          className="chatbot-window"
          style={{
            position: 'fixed',
            bottom: '30px',
            right: '30px',
            width: '400px',
            height: '500px',
            backgroundColor: effectiveColorMode === 'dark' ? '#1a1a2a' : '#ffffff',
            borderRadius: '12px',
            boxShadow: '0 10px 30px rgba(0, 212, 255, 0.3)',
            zIndex: '1000',
            display: 'flex',
            flexDirection: 'column',
            border: effectiveColorMode === 'dark' ? '1px solid rgba(0, 212, 255, 0.3)' : '1px solid #ddd',
            overflow: 'hidden'
          }}
        >
          {/* Header */}
          <div
            style={{
              backgroundColor: effectiveColorMode === 'dark' ? '#00d4ff' : '#0088cc',
              color: effectiveColorMode === 'dark' ? '#0a0a12' : 'white',
              padding: '15px',
              display: 'flex',
              justifyContent: 'space-between',
              alignItems: 'center'
            }}
          >
            <h3 style={{ margin: 0, fontSize: '16px' }}>🤖 AI Assistant</h3>
            <button
              onClick={toggleChat}
              style={{
                background: 'none',
                border: 'none',
                color: effectiveColorMode === 'dark' ? '#0a0a12' : 'white',
                fontSize: '18px',
                cursor: 'pointer',
                padding: '0',
                width: '24px',
                height: '24px'
              }}
            >
              ×
            </button>
          </div>

          {/* Messages */}
          <div
            className="chatbot-messages"
            style={{
              flex: 1,
              padding: '15px',
              overflowY: 'auto',
              display: 'flex',
              flexDirection: 'column',
              gap: '10px',
              backgroundColor: effectiveColorMode === 'dark' ? '#0f0f1a' : '#f9f9f9'
            }}
          >
            {messages.length === 0 ? (
              <div className="welcome-message" style={{ textAlign: 'center', marginTop: '20px' }}>
                <p style={{ color: effectiveColorMode === 'dark' ? '#e0e0ff' : '#333' }}>
                  Hello! I'm your AI assistant for robotics and AI documentation.
                </p>
                <p style={{ color: effectiveColorMode === 'dark' ? '#a0a0ff' : '#666', fontSize: '14px', marginTop: '10px' }}>
                  Ask me anything about ROS 2, NVIDIA Isaac, or other topics:
                </p>
                <div style={{ display: 'flex', flexWrap: 'wrap', gap: '5px', marginTop: '10px', justifyContent: 'center' }}>
                  {quickQuestions.map((question, index) => (
                    <button
                      key={index}
                      onClick={() => handleQuickQuestion(question)}
                      style={{
                        background: effectiveColorMode === 'dark' ? 'rgba(0, 212, 255, 0.2)' : 'rgba(0, 136, 204, 0.1)',
                        border: effectiveColorMode === 'dark' ? '1px solid rgba(0, 212, 255, 0.3)' : '1px solid #0088cc',
                        color: effectiveColorMode === 'dark' ? '#00d4ff' : '#0088cc',
                        borderRadius: '15px',
                        padding: '5px 10px',
                        fontSize: '12px',
                        cursor: 'pointer',
                        whiteSpace: 'nowrap'
                      }}
                      type="button"
                    >
                      {question}
                    </button>
                  ))}
                </div>
              </div>
            ) : (
              messages.map((message) => (
                <div
                  key={message.id}
                  className={`message ${message.sender}`}
                  style={{
                    alignSelf: message.sender === 'user' ? 'flex-end' : 'flex-start',
                    background: message.sender === 'user'
                      ? (effectiveColorMode === 'dark' ? '#00d4ff' : '#0088cc')
                      : (effectiveColorMode === 'dark' ? '#2a2a3a' : '#e9ecef'),
                    color: message.sender === 'user'
                      ? (effectiveColorMode === 'dark' ? '#0a0a12' : 'white')
                      : (effectiveColorMode === 'dark' ? '#e0e0ff' : '#333'),
                    padding: '10px 15px',
                    borderRadius: '18px',
                    maxWidth: '80%',
                    wordWrap: 'break-word'
                  }}
                >
                  {message.text}
                </div>
              ))
            )}
            {isLoading && (
              <div
                className="loading-message"
                style={{
                  alignSelf: 'flex-start',
                  background: effectiveColorMode === 'dark' ? '#2a2a3a' : '#e9ecef',
                  color: effectiveColorMode === 'dark' ? '#e0e0ff' : '#333',
                  padding: '10px 15px',
                  borderRadius: '18px',
                  maxWidth: '80%'
                }}
              >
                🤖 Thinking...
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          {/* Input */}
          <form
            onSubmit={sendMessage}
            style={{
              padding: '15px',
              backgroundColor: effectiveColorMode === 'dark' ? '#1a1a2a' : '#ffffff',
              borderTop: effectiveColorMode === 'dark' ? '1px solid rgba(0, 212, 255, 0.2)' : '1px solid #eee'
            }}
          >
            <div style={{ display: 'flex', gap: '10px' }}>
              <input
                type="text"
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                placeholder="Ask about robotics..."
                style={{
                  flex: 1,
                  padding: '12px 15px',
                  border: effectiveColorMode === 'dark' ? '1px solid rgba(0, 212, 255, 0.3)' : '1px solid #ddd',
                  borderRadius: '20px',
                  backgroundColor: effectiveColorMode === 'dark' ? '#0f0f1a' : '#fff',
                  color: effectiveColorMode === 'dark' ? '#e0e0ff' : '#333',
                  fontSize: '14px'
                }}
                disabled={isLoading}
                aria-label="Type your message"
              />
              <button
                type="submit"
                disabled={!inputValue.trim() || isLoading}
                style={{
                  background: effectiveColorMode === 'dark' ? '#00d4ff' : '#0088cc',
                  color: effectiveColorMode === 'dark' ? '#0a0a12' : 'white',
                  border: 'none',
                  borderRadius: '20px',
                  padding: '12px 20px',
                  cursor: (!inputValue.trim() || isLoading) ? 'not-allowed' : 'pointer',
                  fontSize: '14px',
                  opacity: (!inputValue.trim() || isLoading) ? 0.5 : 1
                }}
              >
                Send
              </button>
            </div>
          </form>
        </div>
      )}
    </div>
  );
};

export default ChatbotComponent;