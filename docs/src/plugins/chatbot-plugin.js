// src/plugins/chatbot-plugin.js
const path = require('path');

module.exports = function (context) {
  const { siteConfig } = context;

  return {
    name: 'chatbot-plugin',

    getClientModules() {
      return [path.resolve(__dirname, '../components/ChatbotComponent.js')];
    },

    injectHtmlTags() {
      return {
        postBodyTags: [
          `<div id="chatbot-root"></div>`,
        ],
      };
    },
  };
};