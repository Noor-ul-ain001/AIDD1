import React from 'react';
import Layout from '@theme-original/Layout';
import FloatingChatbotIcon from '@site/src/components/FloatingChatbotIcon';

export default function LayoutWrapper(props) {
  return (
    <>
      <Layout {...props} />
      <FloatingChatbotIcon />
    </>
  );
}
