import React from 'react';
import { useLocation } from '@docusaurus/router';
import ChatbotComponent from '../components/ChatbotComponent';

// Default implementation, that you can customize
function Root({ children }) {
  const { pathname } = useLocation();

  // Don't show chatbot on admin/build pages
  const shouldShowChatbot = !pathname.includes('/api/') && !pathname.includes('__docusaurus');

  return (
    <>
      {children}
      {shouldShowChatbot && (
        <React.Suspense fallback={<></>}>
          <ChatbotComponent />
        </React.Suspense>
      )}
    </>
  );
}

export default Root;