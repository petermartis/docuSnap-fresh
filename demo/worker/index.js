/**
 * docuSnap — ID R&D liveness CORS proxy
 * Cloudflare Worker (free tier, ~25 lines)
 *
 * Deploy:
 *   1. wrangler deploy  (or paste into the Cloudflare dashboard)
 *
 * Secrets (set in dashboard or via wrangler secret put IDRND_API_KEY):
 *   IDRND_API_KEY  — your ID R&D API key
 */

const IDRND_URL = 'https://idlivedoc-rest-api.idrnd.net/check_liveness';

const CORS = {
  'Access-Control-Allow-Origin':  '*',
  'Access-Control-Allow-Methods': 'POST, OPTIONS',
  'Access-Control-Allow-Headers': 'Content-Type',
};

export default {
  async fetch(request, env) {
    // CORS preflight
    if (request.method === 'OPTIONS') {
      return new Response(null, { status: 204, headers: CORS });
    }

    if (request.method !== 'POST' || new URL(request.url).pathname !== '/check_liveness') {
      return new Response('Not found. POST /check_liveness', { status: 404, headers: CORS });
    }

    const body = await request.arrayBuffer();

    const upstream = await fetch(IDRND_URL, {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json',
        'x-api-key':    env.IDRND_API_KEY,
      },
      body,
    });

    const data = await upstream.arrayBuffer();
    return new Response(data, {
      status:  upstream.status,
      headers: { 'Content-Type': 'application/json', ...CORS },
    });
  },
};
