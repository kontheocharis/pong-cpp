#include <SFML/Graphics.hpp>
#include <vector>
#include <algorithm>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <memory>

namespace eig = Eigen;

struct Settings
{
    int window_width, window_height;
    double paddle_speed;
    double ball_speed;
    double ball_radius;
    sf::Color color;
    sf::Font font;
    double paddle_width, paddle_height;
    double margin_v, margin_h;
};

struct Score
{
    int left, right = 0;
};

namespace Keymap
{
    const auto LeftUp = sf::Keyboard::E;
    const auto LeftDown = sf::Keyboard::D;
    const auto RightUp = sf::Keyboard::O;
    const auto RightDown = sf::Keyboard::K;
};

struct Paddle
{
    double height;
    double position; // of top corner/s
    double velocity;
};

struct Ball
{
    double radius;
    eig::Vector2d position;
    eig::Vector2d velocity;
};

struct Game
{
    std::pair<Paddle, Paddle> paddles;
    Ball ball;

    Score score;
};

int sign(double val) { return (0 < val) - (val < 0); }

void accelerate_paddle(Paddle& paddle, int direction, double dt, const Settings& settings)
{
    assert(direction == -1 || direction == 1);


    if (abs(paddle.velocity) == settings.paddle_speed && sign(paddle.velocity) == direction) return;

    constexpr double acceleration = 100.0;
    const double new_velocity = paddle.velocity + direction * acceleration * dt;

    paddle.velocity = (abs(new_velocity) < settings.paddle_speed) ? new_velocity : settings.paddle_speed * direction;
}

void decelerate_paddle(Paddle& paddle, double dt)
{
    if (paddle.velocity == 0) return;

    constexpr double deceleration = 100.0;
    const double new_velocity = abs(paddle.velocity) - deceleration * dt;

    paddle.velocity = std::max(0.0, new_velocity) * sign(paddle.velocity);
}

bool paddle_can_move(const Paddle& p, int direction, const Settings& settings)
{
    if (p.position <= 0 && direction == -1) return false;
    else if (p.position >= 1 - settings.paddle_height && direction == +1) return false;
    return true;
}

void handle_keypress(Game& game, double dt, const Settings& settings)
{
    if (sf::Keyboard::isKeyPressed(Keymap::LeftDown))
    {
        accelerate_paddle(game.paddles.first, 1, dt, settings);
    }
    else if (sf::Keyboard::isKeyPressed(Keymap::LeftUp))
    {
        accelerate_paddle(game.paddles.first, -1, dt, settings);
    }
    else decelerate_paddle(game.paddles.first, dt);

    if (sf::Keyboard::isKeyPressed(Keymap::RightDown))
    {
        accelerate_paddle(game.paddles.second, 1, dt, settings);
    }
    else if (sf::Keyboard::isKeyPressed(Keymap::RightUp))
    {
        accelerate_paddle(game.paddles.second, -1, dt, settings);
    } else decelerate_paddle(game.paddles.second, dt);
}

std::vector<std::unique_ptr<sf::Shape>> create_game_shapes(const Game& game, const Settings& settings)
{
    std::vector<std::unique_ptr<sf::Shape>> shapes = {};

    {
        const auto radius = game.ball.radius * std::min(settings.window_height, settings.window_width);
        auto ball_shape = std::make_unique<sf::CircleShape>(radius);
        ball_shape->setFillColor(settings.color);
        ball_shape->setOrigin(radius, radius);
        ball_shape->setPosition(
                (game.ball.position.x() * (1 - 2 * settings.margin_h) + settings.margin_h) * settings.window_width,
                (game.ball.position.y() * (1 - 2 * settings.margin_v) + settings.margin_v) * settings.window_height);
        shapes.push_back(std::move(ball_shape));
    }

    {
        auto left_paddle_shape = std::make_unique<sf::RectangleShape>(sf::Vector2f(
            settings.paddle_width * (1 - 2 * settings.margin_h) * settings.window_width,
            settings.paddle_height * (1 - 2 * settings.margin_v) * settings.window_height
        ));

        auto right_paddle_shape = std::make_unique<sf::RectangleShape>(*left_paddle_shape);

        left_paddle_shape->setPosition(
                (settings.margin_h) * settings.window_width,
                (game.paddles.first.position * (1 - 2 * settings.margin_v) + settings.margin_v) * settings.window_height);

        right_paddle_shape->setPosition(
                (1 - settings.margin_h - settings.paddle_width) * settings.window_width,
                (game.paddles.second.position * (1 - 2 * settings.margin_v) + settings.margin_v) * settings.window_height);

        shapes.push_back(std::move(left_paddle_shape));
        shapes.push_back(std::move(right_paddle_shape));
    }

    return shapes;
};

Game reset_game(const Settings& settings, Score score = { 0, 0 }, bool left_start = false)
{ 
    const auto offset = settings.ball_radius + settings.margin_h + settings.paddle_width;
    Ball ball = left_start 
        ? Ball { settings.ball_radius, { 0 + offset, 0.5 }, { settings.ball_speed, 0 } } 
        : Ball { settings.ball_radius, { 1 - offset, 0.5 }, { -settings.ball_speed, 0 } };

    return { 
        { 
            Paddle { settings.paddle_height, 0.5 - settings.paddle_height / 2, 0 },
            Paddle { settings.paddle_height, 0.5 - settings.paddle_height / 2, 0 } 
        },
        ball, score
    };
}

void perform_ball_collisions(Game& game, const Settings& settings)
{
    eig::Vector2d unit_normal { 0, 0 };
    bool need_to_wrap = false;

    if (game.ball.position.y() + game.ball.radius > 1)
    {
        need_to_wrap = true;
        unit_normal += eig::Vector2d { 0, -1 };
        game.ball.position.y() = 1 - game.ball.radius;
    } 
    else if (game.ball.position.y() - game.ball.radius < 0)
    {
        need_to_wrap = true;
        unit_normal += eig::Vector2d { 0, 1 };
        game.ball.position.y() = game.ball.radius;
    }

    if (need_to_wrap)
    {
        unit_normal.normalize();
        game.ball.velocity = (game.ball.velocity - 2 * (game.ball.velocity.dot(unit_normal)) * unit_normal);
    }

    constexpr double bounce_buffer = 0.01;
    constexpr double max_bounce_angle = 5 * M_PI / 12;

    const bool left_win = game.ball.position.y() + game.ball.radius + bounce_buffer < game.paddles.second.position ||
                game.ball.position.y() - game.ball.radius - bounce_buffer > game.paddles.second.position
                + game.paddles.second.height;

    const bool right_win = game.ball.position.y() + game.ball.radius + bounce_buffer < game.paddles.first.position ||
                game.ball.position.y() - game.ball.radius - bounce_buffer > game.paddles.first.position
                + game.paddles.first.height;


    if (game.ball.position.x() + game.ball.radius >= 1 - settings.paddle_width && !left_win)
    {
        game.ball.position.x() = 1 - settings.paddle_width - game.ball.radius;

        const auto relative_isct_y = ((game.paddles.second.position + (game.paddles.second.height) / 2) - game.ball.position.y()) / ((game.paddles.second.height) / 2);
        const auto bounce_angle = relative_isct_y * max_bounce_angle;
        game.ball.velocity = eig::Rotation2Dd(bounce_angle).toRotationMatrix() * eig::Vector2d(-1, 0) * settings.ball_speed;
    } 
    else if (game.ball.position.x() - game.ball.radius <= settings.paddle_width && !right_win)
    {
        game.ball.position.x() = settings.paddle_width + game.ball.radius;

        const auto relative_isct_y = ((game.paddles.first.position + (game.paddles.first.height) / 2) - game.ball.position.y()) / ((game.paddles.first.height) / 2);
        const auto bounce_angle = relative_isct_y * max_bounce_angle;
        game.ball.velocity = eig::Rotation2Dd(-bounce_angle).toRotationMatrix() * eig::Vector2d(1, 0) * settings.ball_speed;
    } 

    if (game.ball.position.x() + game.ball.radius >= 1)
    {
        game.score.left++;
        game = reset_game(settings, game.score, true);
    }
    else if (game.ball.position.x() - game.ball.radius <= 0)
    {
        game.score.right++;
        game = reset_game(settings, game.score, false);
    }
}

void loop_game(Game& game, double dt, const Settings& settings)
{

    game.ball.position += game.ball.velocity * dt;

    handle_keypress(game, dt, settings);

    auto update_position = [&](Paddle& p) {
        int direction = sign(p.velocity);
        if (paddle_can_move(p, direction, settings))
            p.position = p.position + p.velocity * dt;

        else if (direction == -1) p.position = 0;
        else if (direction == +1) p.position = 1 - settings.paddle_height;
    };

    update_position(game.paddles.first);
    update_position(game.paddles.second);

    perform_ball_collisions(game, settings);
}

sf::Text create_score_text(const Game& game, const Settings& settings)
{
    sf::Text text;

    text.setFont(settings.font);
    text.setString(std::to_string(game.score.left) + " - " + std::to_string(game.score.right));
    text.setFillColor(settings.color);

    sf::FloatRect text_rect = text.getLocalBounds();
    text.setOrigin(text_rect.left + text_rect.width / 2,
                   text_rect.top  + text_rect.height / 2);

    text.setPosition(settings.window_width / 2, (settings.margin_v + 0.05) * settings.window_height);

    return text;
}

sf::RectangleShape create_border(const Settings& settings)
{
    sf::RectangleShape border(sf::Vector2f(
            (1 - 2 * settings.margin_h) * settings.window_width,
            (1 - 2 * settings.margin_v) * settings.window_height
            ));
    border.setFillColor(sf::Color::Transparent);
    border.setOutlineThickness(0.0001 * settings.window_width);
    border.setOutlineColor(settings.color);
    border.setPosition(settings.margin_h * settings.window_width, settings.margin_v * settings.window_height);
    return border;
}

int main(int argc, char *argv[])
{
    std::vector<std::string> args(argv + 1, argv + argc);

    Settings settings;

    if (args.size() == 2) {
        try {
            settings.window_width = std::stoi(args[0]);
            settings.window_height = std::stoi(args[1]);
        } catch (std::invalid_argument e) {
            std::cout << "Window dimensions invalid." << "\n";
            exit(128);
        }
    } else {
        settings.window_width = sf::VideoMode::getDesktopMode().width;
        settings.window_height = sf::VideoMode::getDesktopMode().height;
    }

    settings.paddle_width = 0.01;
    settings.paddle_height = 0.2;

    settings.margin_v = 0.05;
    settings.margin_h = 0.05;

    settings.paddle_speed = 1.75;
    settings.ball_speed = 0.8;
    settings.ball_radius = 0.01;
    settings.color = sf::Color::White;

    if (!settings.font.loadFromFile("../fonts/IBMPlexSans-Text.ttf"))
    {
        std::cout << "Cannot find font." << "\n";
        exit(128);
    }

    sf::ContextSettings ctx_settings;
    ctx_settings.antialiasingLevel = 8;

    sf::RenderWindow window(
            sf::VideoMode(settings.window_width, settings.window_height),
            "Pong++",
            sf::Style::Default,
            ctx_settings);
    window.setVerticalSyncEnabled(true);
    sf::Clock clock;

    auto game = reset_game(settings);

    while (window.isOpen())
    {
        sf::Time delta_time = clock.restart();
        sf::Event event;
        while (window.pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                window.close();

            else if (event.type == sf::Event::Resized)
            {
                sf::FloatRect visible_area(0, 0, event.size.width, event.size.height);
                window.setView(sf::View(visible_area));
            }
        }

        const double dt = delta_time.asSeconds();
        loop_game(game, dt, settings);

        window.clear(sf::Color::Black);
        window.draw(create_border(settings));

        for (auto& shape : create_game_shapes(game, settings)) { window.draw(*shape); }

        window.draw(create_score_text(game, settings));

        window.display();
    }

    return 0;
}
