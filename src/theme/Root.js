import React from 'react';
import ChatBot from '../components/ChatBot';

// Root component is rendered on every page
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatBot />
    </>
  );
}