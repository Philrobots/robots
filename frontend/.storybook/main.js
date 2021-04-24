const path = require("path");

module.exports = {
  stories: ["../src/**/*.stories.ts"],
  addons: ["@storybook/addon-actions", "@storybook/addon-knobs", '@storybook/addon-essentials'],
  webpackFinal: async (config) => {
    config.resolve.alias = {
      ...config.resolve.alias,
      "@": path.resolve(__dirname, "../src/"),
    };
    config.resolve.extensions.push(".ts", ".tsx");

    return config;
  },
};
